/*
 * simulator.cpp
 * HRVO Library
 *
 * Copyright 2009 University of North Carolina at Chapel Hill
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jamie Snape, Jur van den Berg, Stephen J. Guy, and Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <https://gamma.cs.unc.edu/HRVO/>
 */

#include "extlibs/hrvo/simulator.h"

#include <stdexcept>

#include "extlibs/hrvo/agent.h"
#include "extlibs/hrvo/goal.h"
#include "extlibs/hrvo/hrvo_agent.h"
#include "extlibs/hrvo/kd_tree.h"
#include "extlibs/hrvo/linear_velocity_agent.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/intersection.h"

Simulator::Simulator(float time_step)
    : globalTime_(0.0f),
      timeStep_(time_step),
      reachedGoals_(false),
      kdTree_(std::make_unique<KdTree>(this))
{
}

void Simulator::updateWorld(const World &world)
{
    // Reset all agents
    agents_.clear();
    friendly_robot_id_map.clear();

    int max_neighbors =
        std::max(1, static_cast<int>(world.friendlyTeam().getAllRobots().size()) - 1);
    for (const Robot &friendly_robot : world.friendlyTeam().getAllRobots())
    {
        std::size_t agent_index = addHRVORobotAgent(friendly_robot, max_neighbors);
        friendly_robot_id_map.emplace(friendly_robot.id(), agent_index);
    }

    for (const Robot &enemy_robot : world.enemyTeam().getAllRobots())
    {
        // Set goal of enemy robot to be the farthest point, when moving in the current
        // direction
        Segment segment(enemy_robot.position(),
                        enemy_robot.position() + enemy_robot.velocity() * 100);

        // Enemy robot should not enter the friendly defense area
        std::unordered_set<Point> intersection_point_set =
            intersection(world.field().friendlyDefenseArea(), segment);
        if (intersection_point_set.empty() &&
            contains(world.field().fieldLines(), enemy_robot.position()))
        {
            // If the robot is in the field, then move in the current direction
            // towards the field edge
            intersection_point_set = intersection(world.field().fieldLines(), segment);
        }

        if (intersection_point_set.empty())
        {
            // If there is no intersection point (robot is outside the field), continue
            // moving in the current direction
            intersection_point_set.insert(enemy_robot.position() +
                                          enemy_robot.velocity() * 5);
        }

        Vector goal_position(static_cast<float>(intersection_point_set.begin()->x()),
                             static_cast<float>(intersection_point_set.begin()->y()));
        addLinearVelocityRobotAgent(enemy_robot, goal_position);
    }

    if (add_ball_agent)
    {
        // Ball should be treated as an agent (obstacle)
        const Ball &ball = world.ball();
        Vector position(ball.position().x(), ball.position().y());
        Vector velocity(ball.velocity().x(), ball.velocity().y());
        Vector goal_pos    = position + 100 * velocity;
        float acceleration = ball.acceleration().length();
        // Minimum of 0.5-meter distance away from the ball, if the ball is an obstacle
        float ball_radius = 0.5f + BALL_AGENT_RADIUS_OFFSET;

        addLinearVelocityAgent(position, ball_radius, velocity, velocity.length(),
                               acceleration, addGoal(goal_pos), 0.1f);
    }
}

void Simulator::updatePrimitiveSet(const TbotsProto::PrimitiveSet &primitive_set)
{
    primitive_set_ = primitive_set;

    add_ball_agent = primitive_set.stay_away_from_ball();

    // Update all friendly agent's goal points based on the matching robot's primitive
    for (auto &[robot_id, primitive] : primitive_set.robot_primitives())
    {
        auto agent_index_iter = friendly_robot_id_map.find(robot_id);
        if (agent_index_iter != friendly_robot_id_map.end())
        {
            unsigned int agent_index = agent_index_iter->second;
            if (agent_index < agents_.size())
            {
                std::unique_ptr<Goal> &goal =
                    goals_[agents_[agent_index]->getGoalIndex()];
                goal->positions_.clear();

                if (primitive.has_move())
                {
                    // TODO (#2418): Update implementation of Primitive to support
                    // multiple path points
                    auto destination = primitive.move().path().point().at(0);
                    goal->positions_.emplace_back(
                        static_cast<float>(destination.x_meters()),
                        static_cast<float>(destination.y_meters()));
                }
            }
        }
    }
}

std::size_t Simulator::addHRVORobotAgent(const Robot &robot, int max_neighbors)
{
    Vector position(static_cast<float>(robot.position().x()),
                    static_cast<float>(robot.position().y()));
    Vector velocity;
    float agent_radius = ROBOT_MAX_RADIUS_METERS * FRIENDLY_ROBOT_RADIUS_SCALE;
    float max_accel    = 1e-4;
    float pref_speed   = 1e-4;
    float max_speed    = 1e-4;

    const std::set<RobotCapability> &unavailable_capabilities =
        robot.getUnavailableCapabilities();
    bool can_move = unavailable_capabilities.find(RobotCapability::Move) ==
                    unavailable_capabilities.end();
    if (can_move)
    {
        velocity   = Vector(static_cast<float>(robot.velocity().x()),
                          static_cast<float>(robot.velocity().y()));
        max_accel  = robot.robotConstants().robot_max_acceleration_m_per_s_2;
        max_speed  = robot.robotConstants().robot_max_speed_m_per_s;
        pref_speed = max_speed * PREF_SPEED_SCALE;
    }

    // Max distance which the robot can travel in one time step + scaling
    float goal_radius        = (max_speed * timeStep_) / 2 * GOAL_RADIUS_SCALE;
    float uncertainty_offset = 0.f;

    // Get this robot's destination point, if it has a primitive
    // If this robot does not have a primitive, then set its current position as its
    // destination
    Vector destination_point   = position;
    auto &robot_primitives     = *primitive_set_.mutable_robot_primitives();
    const auto &primitive_iter = robot_primitives.find(robot.id());
    if (primitive_iter != robot_primitives.end())
    {
        TbotsProto::Primitive primitive = primitive_iter->second;
        TbotsProto::Point destination_point_proto;

        if (primitive.has_move())
        {
            destination_point_proto = primitive.mutable_move()->path().point().at(0);
            destination_point =
                Vector(static_cast<float>(destination_point_proto.x_meters()),
                       static_cast<float>(destination_point_proto.y_meters()));
        }
    }

    return addHRVOAgent(position, agent_radius, velocity, max_speed, pref_speed,
                        max_accel, addGoal(destination_point), goal_radius,
                        MAX_NEIGHBOR_SEARCH_DIST, max_neighbors, uncertainty_offset);
}

std::size_t Simulator::addLinearVelocityRobotAgent(const Robot &robot,
                                                   const Vector &destination)
{
    // TODO (#2371): Replace Vector with Vector
    Vector position(static_cast<float>(robot.position().x()),
                    static_cast<float>(robot.position().y()));
    Vector velocity(static_cast<float>(robot.velocity().x()),
                    static_cast<float>(robot.velocity().y()));
    float max_accel = 0.f;
    float max_speed = robot.robotConstants().robot_max_speed_m_per_s;

    // Max distance which the robot can travel in one time step + scaling
    float goal_radius = (max_speed * timeStep_) / 2 * GOAL_RADIUS_SCALE;

    // Enemy agents should appear larger to friendly agents to avoid collision
    float agent_radius = ROBOT_MAX_RADIUS_METERS * ENEMY_ROBOT_RADIUS_SCALE;

    return addLinearVelocityAgent(position, agent_radius, velocity, max_speed, max_accel,
                                  addGoal(destination), goal_radius);
}

std::size_t Simulator::addHRVOAgent(const Vector &position, float agent_radius,
                                    const Vector &curr_velocity, float maxSpeed,
                                    float prefSpeed, float maxAccel,
                                    std::size_t goal_index, float goalRadius,
                                    float neighborDist, std::size_t maxNeighbors,
                                    float uncertaintyOffset)
{
    std::unique_ptr<HRVOAgent> agent = std::make_unique<HRVOAgent>(
        this, position, goal_index, neighborDist, maxNeighbors, agent_radius,
        curr_velocity, maxAccel, goalRadius, prefSpeed, maxSpeed, uncertaintyOffset);
    agents_.push_back(std::move(agent));
    return agents_.size() - 1;
}

size_t Simulator::addLinearVelocityAgent(const Vector &position, float agent_radius,
                                         const Vector &curr_velocity, float max_speed,
                                         float max_accel, size_t goal_index,
                                         float goal_radius)
{
    std::unique_ptr<LinearVelocityAgent> agent = std::make_unique<LinearVelocityAgent>(
        this, position, agent_radius, curr_velocity, max_speed, max_accel, goal_index,
        goal_radius);

    agents_.push_back(std::move(agent));
    return agents_.size() - 1;
}

std::size_t Simulator::addGoal(const Vector &position)
{
    std::unique_ptr<Goal> goal = std::make_unique<Goal>(position);
    goals_.push_back(std::move(goal));

    return goals_.size() - 1;
}

std::size_t Simulator::addGoalPositions(const std::vector<Vector> &positions)
{
    std::unique_ptr<Goal> goal = std::make_unique<Goal>(positions);
    goals_.push_back(std::move(goal));

    return goals_.size() - 1;
}

std::size_t Simulator::addGoalPositions(const std::vector<Vector> &positions,
                                        const std::vector<float> &speedAtPosition)
{
    std::unique_ptr<Goal> goal = std::make_unique<Goal>(positions, speedAtPosition);
    goals_.push_back(std::move(goal));

    return goals_.size() - 1;
}

void Simulator::doStep()
{
    if (kdTree_ == nullptr)
    {
        throw std::runtime_error(
            "Simulation not initialized when attempting to do step.");
    }

    if (timeStep_ == 0.0f)
    {
        throw std::runtime_error("Time step not set when attempting to do step.");
    }

    reachedGoals_ = true;

    kdTree_->build();

    for (auto &agent : agents_)
    {
        agent->computeNewVelocity();
    }

    for (auto &agent : agents_)
    {
        agent->update();
    }

    globalTime_ += timeStep_;
}

float Simulator::getAgentMaxAccel(std::size_t agentNo) const
{
    return agents_[agentNo]->getMaxAccel();
}

Vector Simulator::getAgentPosition(std::size_t agentNo) const
{
    return agents_[agentNo]->getPosition();
}

float Simulator::getAgentRadius(std::size_t agentNo) const
{
    return agents_[agentNo]->getRadius();
}

bool Simulator::hasAgentReachedGoal(std::size_t agentNo) const
{
    return agents_[agentNo]->hasReachedGoal();
}

Vector Simulator::getAgentVelocity(std::size_t agentNo) const
{
    return agents_[agentNo]->getVelocity();
}

Vector Simulator::getAgentPrefVelocity(std::size_t agentNo) const
{
    return agents_[agentNo]->getPrefVelocity();
}

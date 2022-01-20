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
#include "software/geom/algorithms/intersection.h"
#include "software/geom/algorithms/contains.h"

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

    int max_neighbors = std::max(1, static_cast<int>(world.friendlyTeam().getAllRobots().size()) - 1);
    for (const Robot &friendly_robot : world.friendlyTeam().getAllRobots())
    {
        std::size_t agent_index = addHRVORobotAgent(friendly_robot, max_neighbors);
        friendly_robot_id_map.emplace(friendly_robot.id(), agent_index);
    }

    for (const Robot &enemy_robot : world.enemyTeam().getAllRobots())
    {
        // Set goal of enemy robot to be the farthest point, when moving in the current direction
        Segment segment(enemy_robot.position(),
                        enemy_robot.position() + enemy_robot.velocity() * 100);

        // Enemy robot should not enter the friendly defense area
        std::unordered_set<Point> intersection_point_set =
            intersection(world.field().friendlyDefenseArea(), segment);
        if (intersection_point_set.empty())
        {
            if (contains(world.field().fieldLines(), enemy_robot.position()))
            {
                // If the robot is in the field, then move in the current direction towards the field edge
                intersection_point_set = intersection(world.field().fieldLines(), segment);
            }
            else
            {
                // If the robot is outside the field, continue moving in the current direction
                intersection_point_set.insert(enemy_robot.position() + enemy_robot.velocity() * 5);
            }
        }
        Vector2 goal_position(static_cast<float>(intersection_point_set.begin()->x()),
                              static_cast<float>(intersection_point_set.begin()->y()));
        addLinearVelocityRobotAgent(enemy_robot, goal_position);
    }

    // TODO: If Ball is an obstacle, add ball as an Agent aswell
}

std::size_t Simulator::addHRVORobotAgent(const Robot &robot, int max_neighbors)
{
    Vector2 position(static_cast<float>(robot.position().x()),
                     static_cast<float>(robot.position().y()));
    Vector2 velocity;
    float max_accel = 1e-4;
    float max_speed = 1e-4;

    const std::set<RobotCapability> &unavailable_capabilities = robot.getUnavailableCapabilities();
    bool can_move = unavailable_capabilities.find(RobotCapability::Move) == unavailable_capabilities.end();
    if (can_move)
    {
        velocity = Vector2(static_cast<float>(robot.velocity().x()),
                         static_cast<float>(robot.velocity().y()));
        max_accel = robot.robotConstants().robot_max_acceleration_m_per_s_2;
        max_speed = robot.robotConstants().robot_max_speed_m_per_s;
    }
    // A large distance chosen arbitrarily to allow Agent to decelerate before getting into contact with a neighbor
    float neighbor_dist = 10.f;

    // Max distance which the robot can travel in one time step + scaling
    float goal_radius = (max_speed * timeStep_) / 2 * goal_radius_scale;
    float uncertainty_offset = 0.f;

    // Get this robot's destination point, if it has a primitive
    // If this robot does not have a primitive, then set its current position as its destination
    Vector2 destination_point = position;
    auto &robot_primitives          = *primitive_set_.mutable_robot_primitives();
    const auto primitive_iter = robot_primitives.find(robot.id());
    if (primitive_iter != robot_primitives.end())
    {
        TbotsProto::Primitive primitive = primitive_iter->second;
        TbotsProto::Point destination_point_proto;
        if (primitive.has_move())
        {
            destination_point_proto = primitive.mutable_move()->destination();
            destination_point =
                    Vector2(static_cast<float>(destination_point_proto.x_meters()),
                            static_cast<float>(destination_point_proto.y_meters()));
        }
    }

    std::unique_ptr<HRVOAgent> agent = std::make_unique<HRVOAgent>(
        this, position, addGoal(destination_point), neighbor_dist, max_neighbors,
        ROBOT_MAX_RADIUS_METERS, velocity, max_accel, goal_radius, max_speed, max_speed,
        uncertainty_offset);
    agents_.push_back(std::move(agent));
    return agents_.size() - 1;
}

std::size_t Simulator::addLinearVelocityRobotAgent(const Robot &robot,
                                                   const Vector2 &destination)
{
    // TODO (#2371): Replace Vector2 with Vector
    Vector2 position(static_cast<float>(robot.position().x()),
                     static_cast<float>(robot.position().y()));
    Vector2 velocity(static_cast<float>(robot.velocity().x()),
                     static_cast<float>(robot.velocity().y()));
    float max_accel = robot.robotConstants().robot_max_acceleration_m_per_s_2;
    float max_speed = robot.robotConstants().robot_max_speed_m_per_s;

    // How much larger should the goal radius be. This is added as a safety tolerance so robots
    // do not "teleport" over the goal between simulation frames.
    const float goal_radius_scale = 1.05f;
    // Max distance which the robot can travel in one time step + scaling
    float goal_radius = (max_speed * timeStep_) / 2 * goal_radius_scale;

    // Enemy agents should appear larger to friendly agents to avoid collision
    float agent_radius = ROBOT_MAX_RADIUS_METERS * enemy_robot_radius_scale;

    std::unique_ptr<LinearVelocityAgent> agent = std::make_unique<LinearVelocityAgent>(
        this, position, agent_radius, velocity, max_speed, max_accel,
        addGoal(destination), goal_radius);

    agents_.push_back(std::move(agent));
    return agents_.size() - 1;
}

void Simulator::updatePrimitiveSet(const TbotsProto::PrimitiveSet &primitive_set)
{
    primitive_set_ = primitive_set;
    for (auto &[robot_id, primitive] : primitive_set.robot_primitives())
    {
        // TODO: Update implementation of Primitive to support multiple path points
        if (!primitive.has_move())
        {
            continue;
        }

        auto agent_index_iter = friendly_robot_id_map.find(robot_id);
        if (agent_index_iter != friendly_robot_id_map.end())
        {
            unsigned int agent_index = agent_index_iter->second;
            if (agent_index < agents_.size())
            {
                std::unique_ptr<Goal> &goal = goals_[agents_[agent_index]->goalNo_];
                goal->positions_.clear();

                goal->positions_.push_back(Vector2(
                    static_cast<float>(primitive.move().destination().x_meters()),
                    static_cast<float>(primitive.move().destination().y_meters())));
            }
        }
    }
}

std::size_t Simulator::addHRVOAgent(const Vector2 &position, std::size_t goalNo,
                                    float neighborDist, std::size_t maxNeighbors,
                                    float radius, float goalRadius, float prefSpeed,
                                    float maxSpeed, float uncertaintyOffset,
                                    float maxAccel, const Vector2 &velocity)
{
    // TODO: makeUniquePtr<HRVOAgent>
    std::unique_ptr<HRVOAgent> agent = std::make_unique<HRVOAgent>(
        this, position, goalNo, neighborDist, maxNeighbors, radius, velocity, maxAccel,
        goalRadius, prefSpeed, maxSpeed, uncertaintyOffset);
    agents_.push_back(std::move(agent));
    return agents_.size() - 1;
}

std::size_t Simulator::addGoal(const Vector2 &position)
{
    std::unique_ptr<Goal> goal = std::make_unique<Goal>(position);
    goals_.push_back(std::move(goal));

    return goals_.size() - 1;
}

std::size_t Simulator::addGoalPositions(const std::vector<Vector2> &positions)
{
    std::unique_ptr<Goal> goal = std::make_unique<Goal>(positions);
    goals_.push_back(std::move(goal));

    return goals_.size() - 1;
}

std::size_t Simulator::addGoalPositions(const std::vector<Vector2> &positions,
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

std::size_t Simulator::getAgentGoal(std::size_t agentNo) const
{
    return agents_[agentNo]->goalNo_;
}

float Simulator::getAgentGoalRadius(std::size_t agentNo) const
{
    return agents_[agentNo]->goalRadius_;
}

float Simulator::getAgentMaxAccel(std::size_t agentNo) const
{
    return agents_[agentNo]->maxAccel_;
}

Vector2 Simulator::getAgentPosition(std::size_t agentNo) const
{
    return agents_[agentNo]->position_;
}

float Simulator::getAgentRadius(std::size_t agentNo) const
{
    return agents_[agentNo]->radius_;
}

bool Simulator::getAgentReachedGoal(std::size_t agentNo) const
{
    return agents_[agentNo]->reachedGoal_;
}

Vector2 Simulator::getAgentVelocity(std::size_t agentNo) const
{
    return agents_[agentNo]->velocity_;
}

Vector2 Simulator::getGoalPosition(std::size_t goalNo) const
{
    return goals_[goalNo]->position_;
}

void Simulator::setAgentGoal(std::size_t agentNo, std::size_t goalNo)
{
    agents_[agentNo]->goalNo_ = goalNo;
}

void Simulator::setAgentGoalPosition(std::size_t agentNo, Vector2 position)
{
    goals_[agentNo]->position_ = position;
}

void Simulator::setAgentGoalRadius(std::size_t agentNo, float goalRadius)
{
    agents_[agentNo]->goalRadius_ = goalRadius;
}

void Simulator::setAgentMaxAccel(std::size_t agentNo, float maxAccel)
{
    agents_[agentNo]->maxAccel_ = maxAccel;
}

void Simulator::setAgentMaxSpeed(std::size_t agentNo, float maxSpeed)
{
    agents_[agentNo]->maxSpeed_ = maxSpeed;
}

void Simulator::setAgentPosition(std::size_t agentNo, const Vector2 &position)
{
    agents_[agentNo]->position_ = position;
}

void Simulator::setAgentRadius(std::size_t agentNo, float radius)
{
    agents_[agentNo]->radius_ = radius;
}

void Simulator::setAgentVelocity(std::size_t agentNo, const Vector2 &velocity)
{
    agents_[agentNo]->velocity_ = velocity;
}

Vector2 Simulator::getAgentPrefVelocity(std::size_t agentNo) const
{
    return agents_[agentNo]->prefVelocity_;
}

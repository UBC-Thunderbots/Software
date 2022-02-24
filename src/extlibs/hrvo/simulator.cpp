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
#include "software/logger/logger.h"

HRVOSimulator::HRVOSimulator(float time_step, const RobotConstants_t &robot_constants)
    : global_time(0.0f),
      time_step(time_step),
      robot_constants(robot_constants),
      reached_goals(false),
      kd_tree(std::make_unique<KdTree>(this))
{
}

void HRVOSimulator::updateWorld(const World &world)
{
    const auto &friendly_team = world.friendlyTeam().getAllRobots();
    const auto &enemy_team    = world.enemyTeam().getAllRobots();
    // TODO: Update implementation here to add and remove agents to match the world
    if (friendly_robot_id_map.empty() && enemy_robot_id_map.empty())
    {
        for (const Robot &friendly_robot : friendly_team)
        {
            std::size_t agent_index = addHRVORobotAgent(friendly_robot, MAX_NEIGHBORS);
            friendly_robot_id_map.emplace(friendly_robot.id(), agent_index);
        }

        for (const Robot &enemy_robot : enemy_team)
        {
            // Set goal of enemy robot to be the farthest point, when moving in the
            // current direction
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
                intersection_point_set =
                    intersection(world.field().fieldLines(), segment);
            }

            if (intersection_point_set.empty())
            {
                // If there is no intersection point (robot is outside the field),
                // continue moving in the current direction
                intersection_point_set.insert(enemy_robot.position() +
                                              enemy_robot.velocity() * 5);
            }

            Vector2 goal_position(
                static_cast<float>(intersection_point_set.begin()->x()),
                static_cast<float>(intersection_point_set.begin()->y()));
            std::size_t agent_index =
                addLinearVelocityRobotAgent(enemy_robot, goal_position);
            enemy_robot_id_map.emplace(enemy_robot.id(), agent_index);
        }
    }
    else
    {
        // Update Agents
        for (const Robot &friendly_robot : friendly_team)
        {
            auto hrvo_agent = getFriendlyAgentFromRobotId(friendly_robot.id());
            if (hrvo_agent.has_value())
            {
                Point position = friendly_robot.position();
                hrvo_agent.value()->setPosition(Vector2(position.x(), position.y()));

                // Update velocity every TIME_TO_UPDATE_WORLD seconds to allow for sensor
                // fusion to get an updated velocity.
                if (global_time - last_time_velocity_updated >= TIME_TO_UPDATE_WORLD)
                {
                    Vector velocity = friendly_robot.velocity();
                    hrvo_agent.value()->setVelocity(Vector2(velocity.x(), velocity.y()));
                    last_time_velocity_updated = global_time;
                }
            }
        }

        for (const Robot &enemy_robot : enemy_team)
        {
            auto agent_index_iter = enemy_robot_id_map.find(enemy_robot.id());
            if (agent_index_iter != enemy_robot_id_map.end())
            {
                unsigned int agent_index = agent_index_iter->second;
                Point position           = enemy_robot.position();
                agents[agent_index]->setPosition(Vector2(position.x(), position.y()));

                Vector velocity = enemy_robot.velocity();
                agents[agent_index]->setVelocity(Vector2(velocity.x(), velocity.y()));
            }
            else
            {
                // Robot is new
            }
        }
    }

    // TODO: Dynamically add and remove the ball as an Agent
    //       + Add a ball radius to primitive.proto
    if (add_ball_agent)
    {
        if (ball_agent_id == -1)
        {
            // Ball should be treated as an agent (obstacle)
            const Ball &ball = world.ball();
            Vector2 position(ball.position().x(), ball.position().y());
            Vector2 velocity(ball.velocity().x(), ball.velocity().y());
            Vector2 goal_pos   = position + 100 * velocity;
            float acceleration = ball.acceleration().length();
            // Minimum of 0.5-meter distance away from the ball, if the ball is an
            // obstacle
            float ball_radius = 0.5f + BALL_AGENT_RADIUS_OFFSET;

            std::size_t agent_index =
                addLinearVelocityAgent(position, ball_radius, velocity, abs(velocity),
                                       acceleration, addGoal(goal_pos), 0.1f);
            ball_agent_id = agent_index;
        }
        else
        {
            Point position = world.ball().position();
            agents[ball_agent_id]->setPosition(Vector2(position.x(), position.y()));
        }
    }
}

void HRVOSimulator::updatePrimitiveSet(const TbotsProto::PrimitiveSet &new_primitive_set)
{
    primitive_set = new_primitive_set;

    add_ball_agent = primitive_set.stay_away_from_ball();

    // Update all friendly agent's goal points based on the matching robot's primitive
    for (auto &[robot_id, primitive] : primitive_set.robot_primitives())
    {
        auto hrvo_agent = getFriendlyAgentFromRobotId(robot_id);
        if (hrvo_agent.has_value())
        {
            std::unique_ptr<Goal> &goal = goals[hrvo_agent.value()->getGoalIndex()];
            goal->positions_.clear();

            if (primitive.has_move())
            {
                // TODO (#2418): Update implementation of Primitive to support
                // multiple path points
                goal->positions_.emplace_back(
                    static_cast<float>(primitive.move().destination().x_meters()),
                    static_cast<float>(primitive.move().destination().y_meters()));
            }
        }
    }
}

std::size_t HRVOSimulator::addHRVORobotAgent(const Robot &robot, int max_neighbors)
{
    Vector2 position(static_cast<float>(robot.position().x()),
                     static_cast<float>(robot.position().y()));
    Vector2 velocity;
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
        velocity   = Vector2(static_cast<float>(robot.velocity().x()),
                           static_cast<float>(robot.velocity().y()));
        max_accel  = robot_constants.robot_max_acceleration_m_per_s_2;
        max_speed  = robot_constants.robot_max_speed_m_per_s;
        pref_speed = max_speed * PREF_SPEED_SCALE;
    }

    // TODO (#2418): Replace destination point with a list of path points
    // Get this robot's destination point, if it has a primitive
    // If this robot does not have a primitive, then set its current position as its
    // destination
    Vector2 destination_point    = position;
    float speed_at_goal          = 0.f;
    const auto &robot_primitives = primitive_set.robot_primitives();
    auto primitive_iter          = robot_primitives.find(robot.id());
    if (primitive_iter != robot_primitives.end())
    {
        TbotsProto::Primitive primitive = primitive_iter->second;
        TbotsProto::Point destination_point_proto;
        if (primitive.has_move())
        {
            const auto &move_primitive = primitive.move();
            destination_point_proto    = move_primitive.destination();
            destination_point =
                Vector2(static_cast<float>(destination_point_proto.x_meters()),
                        static_cast<float>(destination_point_proto.y_meters()));
            speed_at_goal = move_primitive.final_speed_m_per_s();
            max_speed     = move_primitive.max_speed_m_per_s();
        }
    }

    // Max distance which the robot can travel in one time step + scaling
    float goal_radius        = (max_speed * time_step) / 2 * GOAL_RADIUS_SCALE;
    float uncertainty_offset = 0.f;

    return addHRVOAgent(position, agent_radius, velocity, max_speed, pref_speed,
                        max_accel, addGoalPositions({destination_point}, {speed_at_goal}),
                        goal_radius, MAX_NEIGHBOR_SEARCH_DIST, max_neighbors,
                        uncertainty_offset);
}

std::size_t HRVOSimulator::addLinearVelocityRobotAgent(const Robot &robot,
                                                       const Vector2 &destination)
{
    // TODO (#2371): Replace Vector2 with Vector
    Vector2 position(static_cast<float>(robot.position().x()),
                     static_cast<float>(robot.position().y()));
    Vector2 velocity(static_cast<float>(robot.velocity().x()),
                     static_cast<float>(robot.velocity().y()));
    float max_accel = 0.f;  // TODO: Maybe use robot constant
    float max_speed = robot_constants.robot_max_speed_m_per_s;

    // Max distance which the robot can travel in one time step + scaling
    float goal_radius = (max_speed * time_step) / 2 * GOAL_RADIUS_SCALE;

    // Enemy agents should appear larger to friendly agents to avoid collision
    float agent_radius = ROBOT_MAX_RADIUS_METERS * ENEMY_ROBOT_RADIUS_SCALE;

    return addLinearVelocityAgent(position, agent_radius, velocity, max_speed, max_accel,
                                  addGoal(destination), goal_radius);
}

std::size_t HRVOSimulator::addHRVOAgent(const Vector2 &position, float agent_radius,
                                        const Vector2 &curr_velocity, float maxSpeed,
                                        float prefSpeed, float maxAccel,
                                        std::size_t goal_index, float goalRadius,
                                        float neighborDist, std::size_t maxNeighbors,
                                        float uncertaintyOffset)
{
    std::shared_ptr<HRVOAgent> agent = std::make_shared<HRVOAgent>(
        this, position, goal_index, neighborDist, maxNeighbors, agent_radius,
        curr_velocity, maxAccel, goalRadius, prefSpeed, maxSpeed, uncertaintyOffset);
    agents.push_back(std::move(agent));
    return agents.size() - 1;
}

size_t HRVOSimulator::addLinearVelocityAgent(const Vector2 &position, float agent_radius,
                                             const Vector2 &curr_velocity,
                                             float max_speed, float max_accel,
                                             size_t goal_index, float goal_radius)
{
    std::shared_ptr<LinearVelocityAgent> agent = std::make_shared<LinearVelocityAgent>(
        this, position, agent_radius, curr_velocity, max_speed, max_accel, goal_index,
        goal_radius);

    agents.push_back(std::move(agent));
    return agents.size() - 1;
}

std::size_t HRVOSimulator::addGoal(const Vector2 &position)
{
    std::unique_ptr<Goal> goal = std::make_unique<Goal>(position);
    goals.push_back(std::move(goal));

    return goals.size() - 1;
}

std::size_t HRVOSimulator::addGoalPositions(const std::vector<Vector2> &positions)
{
    std::unique_ptr<Goal> goal = std::make_unique<Goal>(positions);
    goals.push_back(std::move(goal));

    return goals.size() - 1;
}

std::size_t HRVOSimulator::addGoalPositions(const std::vector<Vector2> &positions,
                                            const std::vector<float> &speedAtPosition)
{
    std::unique_ptr<Goal> goal = std::make_unique<Goal>(positions, speedAtPosition);
    goals.push_back(std::move(goal));

    return goals.size() - 1;
}

void HRVOSimulator::doStep()
{
    if (kd_tree == nullptr)
    {
        throw std::runtime_error(
            "Simulation not initialized when attempting to do step.");
    }

    if (time_step == 0.0f)
    {
        throw std::runtime_error("Time step not set when attempting to do step.");
    }

    reached_goals = true;

    if (agents.size() == 0)
    {
        return;
    }

    kd_tree->build();

    for (auto &agent : agents)
    {
        agent->computeNewVelocity();
    }

    for (auto &agent : agents)
    {
        agent->update();
    }

    global_time += time_step;
}

Vector HRVOSimulator::getRobotVelocity(unsigned int robot_id) const
{
    auto hrvo_agent = getFriendlyAgentFromRobotId(robot_id);
    if (hrvo_agent.has_value())
    {
        Vector2 velocity_vector_2 = hrvo_agent.value()->getVelocity();
        return Vector(velocity_vector_2.getX(), velocity_vector_2.getY());
    }
    LOG(WARNING) << "Velocity for robot " << robot_id
                 << " can not be found since it does not exist in HRVO Simulator"
                 << std::endl;
    return Vector();
}

std::vector<Polygon> HRVOSimulator::getRobotVelocityObstacles(unsigned int robot_id) const
{
    std::vector<Polygon> velocity_obstacles;
    auto hrvo_agent = getFriendlyAgentFromRobotId(robot_id);
    if (hrvo_agent.has_value())
    {
        auto agent_position = hrvo_agent.value()->getPosition();
        for (const Agent::VelocityObstacle &vo : hrvo_agent.value()->velocityObstacles_)
        {
            std::vector<Point> points;
            Vector2 shifted_apex  = agent_position + vo.apex_;
            Vector2 shifted_side1 = agent_position + vo.side1_;
            Vector2 shifted_side2 = agent_position + vo.side2_;
            points.emplace_back(Point(shifted_apex.getX(), shifted_apex.getY()));
            points.emplace_back(Point(shifted_side1.getX(), shifted_side1.getY()));
            points.emplace_back(Point(shifted_side2.getX(), shifted_side2.getY()));
            velocity_obstacles.emplace_back(Polygon(points));
        }
    }
    return velocity_obstacles;
}

std::vector<Circle> HRVOSimulator::getRobotCandidateCircles(unsigned int robot_id,
                                                            const float circle_rad) const
{
    std::vector<Circle> candidate_circles;
    auto hrvo_agent = getFriendlyAgentFromRobotId(robot_id);
    if (hrvo_agent.has_value())
    {
        for (auto &candidate : hrvo_agent.value()->candidates_)
        {
            Vector2 candidate_pos =
                hrvo_agent.value()->getPosition() + candidate.second.position_;
            candidate_circles.emplace_back(
                Circle(Point(candidate_pos.getX(), candidate_pos.getY()), circle_rad));
        }
    }
    return candidate_circles;
}

std::optional<std::shared_ptr<HRVOAgent>> HRVOSimulator::getFriendlyAgentFromRobotId(
    unsigned int robot_id) const
{
    auto agent_index_iter = friendly_robot_id_map.find(robot_id);
    if (agent_index_iter != friendly_robot_id_map.end())
    {
        unsigned int agent_index = agent_index_iter->second;
        auto hrvo_agent = std::static_pointer_cast<HRVOAgent>(agents[agent_index]);
        if (hrvo_agent != nullptr)
        {
            return hrvo_agent;
        }
    }
    return std::nullopt;
}

float HRVOSimulator::getAgentMaxAccel(std::size_t agentNo) const
{
    return agents[agentNo]->getMaxAccel();
}

Vector2 HRVOSimulator::getAgentPosition(std::size_t agentNo) const
{
    return agents[agentNo]->getPosition();
}

float HRVOSimulator::getAgentRadius(std::size_t agentNo) const
{
    return agents[agentNo]->getRadius();
}

bool HRVOSimulator::hasAgentReachedGoal(std::size_t agentNo) const
{
    return agents[agentNo]->hasReachedGoal();
}

Vector2 HRVOSimulator::getAgentVelocity(std::size_t agentNo) const
{
    return agents[agentNo]->getVelocity();
}

Vector2 HRVOSimulator::getAgentPrefVelocity(std::size_t agentNo) const
{
    return agents[agentNo]->getPrefVelocity();
}

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
#include "extlibs/hrvo/hrvo_agent.h"
#include "extlibs/hrvo/kd_tree.h"
#include "extlibs/hrvo/linear_velocity_agent.h"
#include "proto/message_translation/tbots_geometry.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/intersection.h"
#include "software/logger/logger.h"

HRVOSimulator::HRVOSimulator(float time_step, const RobotConstants_t &robot_constants,
                             const TeamColour friendly_team_colour)
    : primitive_set(),
      robot_constants(robot_constants),
      global_time(0.0f),
      time_step(time_step),
      reached_goals(false),
      kd_tree(std::make_unique<KdTree>(this)),
      world(std::nullopt),
      agents(),
      friendly_robot_id_map(),
      enemy_robot_id_map(),
      friendly_team_colour(friendly_team_colour)
{
}

void HRVOSimulator::updateWorld(const World &world)
{
    this->world               = world;
    const auto &friendly_team = world.friendlyTeam().getAllRobots();
    const auto &enemy_team    = world.enemyTeam().getAllRobots();
    // TODO (#2498): Update implementation to correctly support adding and removing agents
    //               to represent the newly added and removed friendly/enemy robots in the
    //               World.
    if (friendly_robot_id_map.empty() && enemy_robot_id_map.empty())
    {
        for (const Robot &friendly_robot : friendly_team)
        {
            std::size_t agent_index = addHRVORobotAgent(friendly_robot);
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

            Vector goal_position = intersection_point_set.begin()->toVector();
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
                hrvo_agent.value()->setPosition(friendly_robot.position().toVector());
                // We do not use velocity feedback for friendly robots as it results
                // in the robots not being able to accelerate properly.
            }
        }

        for (const Robot &enemy_robot : enemy_team)
        {
            auto agent_index_iter = enemy_robot_id_map.find(enemy_robot.id());
            if (agent_index_iter != enemy_robot_id_map.end())
            {
                unsigned int agent_index = agent_index_iter->second;
                agents[agent_index]->setPosition(enemy_robot.position().toVector());
                agents[agent_index]->setVelocity(enemy_robot.velocity());
            }
        }
    }
}

void HRVOSimulator::updatePrimitiveSet(const TbotsProto::PrimitiveSet &new_primitive_set)
{
    primitive_set = new_primitive_set;

    // Update all friendly agent's primitives
    for (auto &[robot_id, primitive] : primitive_set.robot_primitives())
    {
        auto hrvo_agent_opt = getFriendlyAgentFromRobotId(robot_id);
        if (hrvo_agent_opt.has_value() && world.has_value())
        {
            hrvo_agent_opt.value()->updatePrimitive(primitive, world.value());
        }
    }
}

std::size_t HRVOSimulator::addHRVORobotAgent(const Robot &robot)
{
    Vector position = robot.position().toVector();
    Vector velocity;
    float max_accel = 1e-4;
    float max_speed = 1e-4;

    const std::set<RobotCapability> &unavailable_capabilities =
        robot.getUnavailableCapabilities();
    bool can_move = unavailable_capabilities.find(RobotCapability::Move) ==
                    unavailable_capabilities.end();
    if (can_move)
    {
        velocity  = Vector(static_cast<float>(robot.velocity().x()),
                          static_cast<float>(robot.velocity().y()));
        max_accel = robot_constants.robot_max_acceleration_m_per_s_2;
        max_speed = robot_constants.robot_max_speed_m_per_s;
    }

    // TODO (#2418): Replace destination point with a list of path points
    // Get this robot's destination point, if it has a primitive
    // If this robot does not have a primitive, then set its current position as its
    // destination
    Vector destination_point     = position;
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
            // TODO (#2418): Update implementation of Primitive to support
            // multiple path points and remove this check
            CHECK(move_primitive.motion_control().path().points().size() >= 2)
                << "Empty path: "
                << move_primitive.motion_control().path().points().size() << std::endl;
            destination_point_proto =
                move_primitive.motion_control().path().points().at(1);
            destination_point =
                Vector(static_cast<float>(destination_point_proto.x_meters()),
                       static_cast<float>(destination_point_proto.y_meters()));
            speed_at_goal = move_primitive.final_speed_m_per_s();
            max_speed     = move_primitive.max_speed_m_per_s();
        }
    }

    // Max distance which the robot can travel in one time step + scaling
    float path_radius = (max_speed * time_step) / 2;

    AgentPath path =
        AgentPath({PathPoint(destination_point, speed_at_goal)}, path_radius);

    return addHRVOAgent(position, ROBOT_MAX_RADIUS_METERS,
                        FRIENDLY_ROBOT_RADIUS_MAX_INFLATION, velocity, max_speed,
                        max_accel, path, MAX_NEIGHBOR_SEARCH_DIST, MAX_NEIGHBORS);
}

std::size_t HRVOSimulator::addLinearVelocityRobotAgent(const Robot &robot,
                                                       const Vector &destination)
{
    // TODO (#2371): Replace Vector with Vector
    Vector position = robot.position().toVector();
    Vector velocity = robot.velocity();
    float max_accel = 0.f;
    float max_speed = robot_constants.robot_max_speed_m_per_s;

    // Max distance which the robot can travel in one time step + scaling
    float path_radius = (max_speed * time_step) / 2;

    AgentPath path = AgentPath({PathPoint(destination, 0.0f)}, path_radius);
    return addLinearVelocityAgent(position, ROBOT_MAX_RADIUS_METERS,
                                  ENEMY_ROBOT_RADIUS_MAX_INFLATION, velocity, max_speed,
                                  max_accel, path);
}

std::size_t HRVOSimulator::addHRVOAgent(const Vector &position, float agent_radius,
                                        float max_radius_inflation,
                                        const Vector &curr_velocity, float max_speed,
                                        float max_accel, AgentPath &path,
                                        float max_neighbor_dist,
                                        std::size_t max_neighbors)
{
    std::shared_ptr<HRVOAgent> agent = std::make_shared<HRVOAgent>(
        this, position, max_neighbor_dist, max_neighbors, agent_radius,
        max_radius_inflation, curr_velocity, max_accel, path, max_speed);
    agents.push_back(std::move(agent));
    return agents.size() - 1;
}

size_t HRVOSimulator::addLinearVelocityAgent(const Vector &position, float agent_radius,
                                             float max_radius_inflation,
                                             const Vector &curr_velocity, float max_speed,
                                             float max_accel, AgentPath &path)
{
    std::shared_ptr<LinearVelocityAgent> agent = std::make_shared<LinearVelocityAgent>(
        this, position, agent_radius, max_radius_inflation, curr_velocity, max_speed,
        max_accel, path);

    agents.push_back(std::move(agent));
    return agents.size() - 1;
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

    // Update all agent radii based on their velocity
    for (auto &agent : agents)
    {
        agent->updateRadiusFromVelocity();
    }

    // Compute what velocity each agent will take next
    for (auto &agent : agents)
    {
        agent->computeNewVelocity();
    }

    // Update the positions of all agents given their velocity
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
        return hrvo_agent.value()->getVelocity();
    }
    LOG(WARNING) << "Velocity for robot " << robot_id
                 << " can not be found since it does not exist in HRVO Simulator"
                 << std::endl;
    return Vector();
}

void HRVOSimulator::visualize(unsigned int robot_id) const
{
    auto friendly_agent_opt = getFriendlyAgentFromRobotId(robot_id);
    if (!friendly_agent_opt.has_value())
    {
        // HRVO friendly agent with robot id can not be visualized
        return;
    }

    TbotsProto::HRVOVisualization hrvo_visualization;
    hrvo_visualization.set_robot_id(robot_id);

    auto friendly_agent = friendly_agent_opt.value();
    auto vo_protos      = friendly_agent->getVelocityObstaclesAsProto();
    *(hrvo_visualization.mutable_velocity_obstacles()) = {vo_protos.begin(),
                                                          vo_protos.end()};

    // Visualize all agents
    for (const auto &agent : agents)
    {
        Point position(agent->getPosition());
        *(hrvo_visualization.add_robots()) =
            *createCircleProto(Circle(position, agent->getRadius()));
    }

    // Visualize the ball obstacle
    if (friendly_agent->ball_obstacle.has_value())
    {
        TbotsProto::Circle ball_circle =
            friendly_agent->ball_obstacle.value()->createObstacleProto().circle()[0];
        *(hrvo_visualization.add_robots()) = ball_circle;
    }

    if (friendly_team_colour == TeamColour::YELLOW)
    {
        LOG(VISUALIZE, YELLOW_HRVO_PATH) << hrvo_visualization;
    }
    else
    {
        LOG(VISUALIZE, BLUE_HRVO_PATH) << hrvo_visualization;
    }
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

float HRVOSimulator::getAgentMaxAccel(std::size_t agent_no) const
{
    return agents[agent_no]->getMaxAccel();
}

Vector HRVOSimulator::getAgentPosition(std::size_t agent_no) const
{
    return agents[agent_no]->getPosition();
}

float HRVOSimulator::getAgentRadius(std::size_t agent_no) const
{
    return agents[agent_no]->getRadius();
}

bool HRVOSimulator::hasAgentReachedGoal(std::size_t agent_no) const
{
    return agents[agent_no]->hasReachedGoal();
}

Vector HRVOSimulator::getAgentVelocity(std::size_t agent_no) const
{
    return agents[agent_no]->getVelocity();
}

Vector HRVOSimulator::getAgentPrefVelocity(std::size_t agent_no) const
{
    return agents[agent_no]->getPrefVelocity();
}

const std::unique_ptr<KdTree> &HRVOSimulator::getKdTree() const
{
    return kd_tree;
}

const std::vector<std::shared_ptr<Agent>> &HRVOSimulator::getAgents() const
{
    return agents;
}

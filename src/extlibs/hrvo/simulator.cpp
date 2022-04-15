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
#include "proto/message_translation/tbots_protobuf.h"
#include "proto/visualization.pb.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/intersection.h"
#include "software/logger/logger.h"

HRVOSimulator::HRVOSimulator(float time_step, const RobotConstants_t &robot_constants)
    : global_time(0.0f),
      last_update_velocities_timestamp(Timestamp::fromSeconds(0)),
      last_update_positions_timestamp(Timestamp::fromSeconds(0)),
      update_velocities_frequency(Duration::fromSeconds(3)),
      update_positions_frequency(Duration::fromSeconds(0.1)),
      time_step(time_step),
      robot_constants(robot_constants),
      reached_goals(false),
      kd_tree(std::make_unique<KdTree>(this))
{
    CHECK(time_step > 0.f);
}

// TODO: Update to 2021 robot constants: simulated_er_force_sim_text_fixture.cpp and
// robot.h
// TODO: Add CallGrind Artifacts to GitHub Actions as job on Push!?
// TODO: Look into chip_tactic (passes, but fragile) and kick_tactic (fails because of that) robot continuing to rotate.
//       MoveGoalieToGoalLineTacticTest.move_to_goal_line_test HRVO agent is ahead of sim robot
// TODO: Replace friendly_robot_id_map (robot_id --> agent_id) with (robot_id -->
// shared_ptr<Agent>) and remove
//       all use cases of AgentNo/agent_id
// NOTE: Robot doesn't stop right at destination at times, since the
// robotReachedDestination transition condition
//       has a threshold of 0.05m.
void HRVOSimulator::updateWorld(const World &world)
{
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

        // HRVO Simulator has a more accurate estimate of the current friendly robot
        // state than the world generated by SensorFusion, as a result, the robot
        // state is only updated every 0.5sec to avoid HRVO Simulator deviating from
        // the real world.
//        if (world.getMostRecentTimestamp() >=
//            last_update_positions_timestamp + update_positions_frequency)
//        {
            last_update_positions_timestamp = world.getMostRecentTimestamp();
            for (const Robot &friendly_robot : friendly_team)
            {
                auto hrvo_agent = getFriendlyAgentFromRobotId(friendly_robot.id());
                if (hrvo_agent.has_value())
                {
                    // TODO: Update velocity every 2 sec? but position every 0.2sec ...
                    //       Worried that if robots have collision, hrvo sim wouldnt take that into account...
                    hrvo_agent.value()->setPosition(friendly_robot.position().toVector());
                }
            }
//        }
//
//        if (world.getMostRecentTimestamp() >=
//            last_update_velocities_timestamp + update_velocities_frequency)
//        {
            last_update_velocities_timestamp = world.getMostRecentTimestamp();
            for (const Robot &friendly_robot : friendly_team)
            {
                auto hrvo_agent = getFriendlyAgentFromRobotId(friendly_robot.id());
                if (hrvo_agent.has_value())
                {
                    hrvo_agent.value()->setVelocity(friendly_robot.velocity());
                }
            }
//        }

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

    // TODO (#2498): Dynamically add and remove the ball as an Agent, and if needed
    //               update its radius based on the PrimitiveSet
    if (add_ball_agent)
    {
        if (ball_agent_id == -1)
        {
            // Ball should be treated as an agent (obstacle)
            const Ball &ball = world.ball();
            Vector position(ball.position().x(), ball.position().y());
            Vector velocity(ball.velocity().x(), ball.velocity().y());
            Vector goal_pos    = position + 100 * velocity;
            float acceleration = ball.acceleration().length();
            // Minimum of 0.5-meter distance away from the ball, if the ball is an
            // obstacle
            float ball_radius = 0.5f + BALL_AGENT_RADIUS_OFFSET;

            AgentPath path          = AgentPath({PathPoint(goal_pos, 0.0f)}, 0.1f);
            std::size_t agent_index = addLinearVelocityAgent(
                position, ball_radius, velocity, velocity.length(), acceleration, path);
            ball_agent_id = agent_index;
        }
        else
        {
            Point position = world.ball().position();
            agents[ball_agent_id]->setPosition(position.toVector());
        }
    }
    else if (ball_agent_id != -1)
    {
        agents[ball_agent_id]->setRadius(0.f);
    }

    // TODO: DEBUGGING!!!!!!!!!!!!!!!!!
    doStep();
    doStep();
//    doStep();
}

void HRVOSimulator::updatePrimitiveSet(const TbotsProto::PrimitiveSet &new_primitive_set)
{
    primitive_set = new_primitive_set;

    // TODO (#2498): Dynamically add and remove the ball as an Agent, and if needed
    //               update its radius based on the PrimitiveSet
    add_ball_agent = primitive_set.stay_away_from_ball();

    // Update all friendly agent's goal points based on the matching robot's primitive
    for (auto &[robot_id, primitive] : primitive_set.robot_primitives())
    {
        auto hrvo_agent_opt = getFriendlyAgentFromRobotId(robot_id);
        if (hrvo_agent_opt.has_value())
        {
            auto hrvo_agent = hrvo_agent_opt.value();
            AgentPath path;

            if (primitive.has_move())
            {
                float speed_at_dest = primitive.move().final_speed_m_per_s();
                float new_max_speed = primitive.move().max_speed_m_per_s();
                hrvo_agent->setMaxSpeed(new_max_speed);
                hrvo_agent->setPreferredSpeed(new_max_speed * PREF_SPEED_SCALE);

                // TODO (#2418): Update implementation of Primitive to support
                // multiple path points
                auto destination = primitive.move().path().point().at(0);

                // Max distance which the robot can travel in one time step + scaling
                float path_radius =
                    (hrvo_agent->getMaxSpeed() * time_step) / 2 * GOAL_RADIUS_SCALE;
                path = AgentPath(
                    {PathPoint(Vector(destination.x_meters(), destination.y_meters()),
                               speed_at_dest)},
                    path_radius);
            }

            hrvo_agent->setPath(path);
        }
    }
}

std::size_t HRVOSimulator::addHRVORobotAgent(const Robot &robot)
{
    Vector position = robot.position().toVector();
    Vector velocity;
    float max_accel  = 1e-4;
    float pref_speed = 1e-4;
    float max_speed  = 1e-4;

    const std::set<RobotCapability> &unavailable_capabilities =
        robot.getUnavailableCapabilities();
    bool can_move = unavailable_capabilities.find(RobotCapability::Move) ==
                    unavailable_capabilities.end();
    if (can_move)
    {
        velocity   = robot.velocity();
        max_accel  = robot_constants.robot_max_acceleration_m_per_s_2;
        max_speed  = robot_constants.robot_max_speed_m_per_s;
        pref_speed = max_speed * PREF_SPEED_SCALE;
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
            destination_point_proto    = move_primitive.path().point().at(0);
            destination_point =
                Vector(static_cast<float>(destination_point_proto.x_meters()),
                       static_cast<float>(destination_point_proto.y_meters()));
            speed_at_goal = move_primitive.final_speed_m_per_s();
            max_speed     = move_primitive.max_speed_m_per_s();
        }
    }

    // Max distance which the robot can travel in one time step + scaling
    float path_radius        = (max_speed * time_step) / 2 * GOAL_RADIUS_SCALE;
    float uncertainty_offset = 0.f;

    AgentPath path =
        AgentPath({PathPoint(destination_point, speed_at_goal)}, path_radius);

    return addHRVOAgent(position, ROBOT_MAX_RADIUS_METERS, velocity, max_speed,
                        pref_speed, max_accel, path, MAX_NEIGHBOR_SEARCH_DIST,
                        MAX_NEIGHBORS, uncertainty_offset);
}

std::size_t HRVOSimulator::addLinearVelocityRobotAgent(const Robot &robot,
                                                       const Vector &destination)
{
    Vector position = robot.position().toVector();
    Vector velocity = robot.velocity();
    float max_accel = 0.f;
    float max_speed = robot_constants.robot_max_speed_m_per_s;

    // Max distance which the robot can travel in one time step + scaling
    float path_radius = (max_speed * time_step) / 2 * GOAL_RADIUS_SCALE;

    AgentPath path = AgentPath({PathPoint(destination, 0.0f)}, path_radius);
    return addLinearVelocityAgent(position, ROBOT_MAX_RADIUS_METERS, velocity, max_speed,
                                  max_accel, path);
}

std::size_t HRVOSimulator::addHRVOAgent(const Vector &position, float agent_radius,
                                        const Vector &curr_velocity, float maxSpeed,
                                        float prefSpeed, float maxAccel, AgentPath &path,
                                        float neighborDist, std::size_t maxNeighbors,
                                        float uncertaintyOffset)
{
    std::shared_ptr<HRVOAgent> agent = std::make_shared<HRVOAgent>(
        this, position, curr_velocity, prefSpeed, maxSpeed, maxAccel, path, agent_radius,
        maxNeighbors, neighborDist, uncertaintyOffset);
    agents.push_back(std::move(agent));
    return agents.size() - 1;
}

size_t HRVOSimulator::addLinearVelocityAgent(const Vector &position, float agent_radius,
                                             const Vector &curr_velocity, float max_speed,
                                             float max_accel, AgentPath &path)
{
    std::shared_ptr<LinearVelocityAgent> agent = std::make_shared<LinearVelocityAgent>(
        this, position, curr_velocity, max_speed, max_accel, path, agent_radius);

    agents.push_back(std::move(agent));
    return agents.size() - 1;
}

void HRVOSimulator::doStep()
{
    //    std::cout << "doStep" << std::endl;
    //    if (kd_tree == nullptr)
    //    {
    //        throw std::runtime_error(
    //                "Simulation not initialized when attempting to do step.");
    //    }

    //    if (time_step == 0.0f)
    //    {
    //        throw std::runtime_error("Time step not set when attempting to do step.");
    //    }

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
        return hrvo_agent.value()->getVelocity();
    }
    LOG(WARNING) << "Velocity for robot " << robot_id
                 << " can not be found since it does not exist in HRVO Simulator"
                 << std::endl;
    return Vector();
}

void HRVOSimulator::visualize(unsigned int robot_id) const
{
    // TODO (#2499): Create a new HRVO visualization proto and uncomment/update
    // LOG(VISUALIZE)
    TbotsProto::Obstacles obstacle_proto;

    // Add velocity obstacles and candidate new velocities to be visualized
    auto friendly_agent_opt = getFriendlyAgentFromRobotId(robot_id);
    if (friendly_agent_opt.has_value())
    {
        auto friendly_agent = friendly_agent_opt.value();
        for (auto &obstacle : friendly_agent->getVelocityObstaclesAsPolygons())
        {
            *(obstacle_proto.add_polygon()) = *createPolygonProto(obstacle);
        }

        //        for (auto &candidate_circle :
        //        friendly_agent->getCandidateVelocitiesAsCircles())
        //        {
        //            *(obstacle_proto.add_circle()) =
        //            *createCircleProto(candidate_circle);
        //        }

        *(obstacle_proto.add_circle()) = *createCircleProto(
            Circle(Point(friendly_agent->prev_vel + friendly_agent->getPosition()),
                   friendly_agent->max_accel_ * time_step));
        *(obstacle_proto.add_circle()) = *createCircleProto(Circle(
            Point(friendly_agent->pref_velocity_ + friendly_agent->getPosition()), 0.03));
        *(obstacle_proto.add_circle()) = *createCircleProto(Circle(
            Point(friendly_agent->new_velocity_ + friendly_agent->getPosition()), 0.05));
        *(obstacle_proto.add_circle()) = *createCircleProto(Circle(
            Point(friendly_agent->velocity_ + friendly_agent->getPosition()), 0.07));
        auto path_point_opt            = friendly_agent->getPath().getCurrentPathPoint();
        if (path_point_opt.has_value())
        {
            *(obstacle_proto.add_circle()) =
                *createCircleProto(Circle(Point(path_point_opt->getPosition()), 0.06));
        }

        LOG(VISUALIZE) << *createNamedValue(
            "hrvo_accel",
            static_cast<float>(
                (friendly_agent->velocity_ - friendly_agent->prev_vel).length() /
                time_step));
        LOG(VISUALIZE) << *createNamedValue(
            "hrvo velocity", static_cast<float>(friendly_agent->velocity_.length()));
        LOG(VISUALIZE) << *createNamedValue(
                    "dist_remaining_to_goal",
                    static_cast<float>(friendly_agent->dist_remaining_to_goal));
        LOG(VISUALIZE) << *createNamedValue(
                    "decel_dist",
                    static_cast<float>(friendly_agent->decel_dist));
        LOG(VISUALIZE) << *createNamedValue(
                    "ideal_speed",
                    static_cast<float>(friendly_agent->ideal_speed));
    }

    // Add circles representing agents
    for (auto &agent : agents)
    {
        Point position(agent->getPosition());
        *(obstacle_proto.add_circle()) =
            *createCircleProto(Circle(position, agent->getRadius()));
    }
    LOG(VISUALIZE) << obstacle_proto;
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

// TODO: Remove all helpers that use agent_id
float HRVOSimulator::getAgentMaxAccel(std::size_t agent_id) const
{
    return agents[agent_id]->getMaxAccel();
}

Vector HRVOSimulator::getAgentPosition(std::size_t agent_id) const
{
    return agents[agent_id]->getPosition();
}

float HRVOSimulator::getAgentRadius(std::size_t agent_id) const
{
    return agents[agent_id]->getRadius();
}

bool HRVOSimulator::hasAgentReachedGoal(std::size_t agent_id) const
{
    return agents[agent_id]->hasReachedGoal();
}

Vector HRVOSimulator::getAgentVelocity(std::size_t agent_id) const
{
    return agents[agent_id]->getVelocity();
}

Vector HRVOSimulator::getAgentPrefVelocity(std::size_t agent_id) const
{
    return agents[agent_id]->getPrefVelocity();
}

const std::unique_ptr<KdTree> &HRVOSimulator::getKdTree() const
{
    return kd_tree;
}

const std::vector<std::shared_ptr<Agent>> &HRVOSimulator::getAgents() const
{
    return agents;
}

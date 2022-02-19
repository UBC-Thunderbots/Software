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

HRVOSimulator::HRVOSimulator(float time_step, const RobotConstants_t &robot_constants)
    : globalTime_(0.0f),
      timeStep_(time_step),
      robot_constants_(robot_constants),
      reachedGoals_(false),
      kdTree_(std::make_unique<KdTree>(this))
{
    bool record_simulator = true;
    if (record_simulator)
    {
        std::string file_directory = "/tmp/";
        output_file_loc = file_directory +  + "simulation_playback.csv";
        output_file = std::ofstream(output_file_loc);
        if (!output_file.is_open())
        {
            std::cout << "File " << output_file_loc << " can not be created and opened."
                      << std::endl;
            return;
        }

        // Column Names
        output_file
                << "frame,time,computation_time,robot_id,radius,x,y,velocity_x,velocity_y,speed,goal_x,goal_y,goal_radius,has_collided,pref_vel_x,pref_vel_y"
                << std::endl;
    }
}

HRVOSimulator::~HRVOSimulator()
{
    if (output_file.is_open())
    {
        output_file.close();
        std::cout << "Test information outputted to " << output_file_loc << std::endl;
    }
}

void HRVOSimulator::updateWorld(const World &world)
{
    /**
     * TODO: update the implementation of updateWorld to update agents, instead of replacing them everytime.
     * 1. Remove Agents that are not in the world anymore
     * 2. Add Agents based on new robots added to the world
     * 3. Update robot positions
     */
    const auto& friendly_team = world.friendlyTeam().getAllRobots();
    const auto& enemy_team = world.enemyTeam().getAllRobots();
    // Update this snippet of code based on above TODO
    if (friendly_robot_id_map.empty() && enemy_robot_id_map.empty())
    {
        int max_neighbors =
            std::max(1, static_cast<int>(world.friendlyTeam().getAllRobots().size()) - 1);
        for (const Robot &friendly_robot : friendly_team)
        {
            std::size_t agent_index = addHRVORobotAgent(friendly_robot, max_neighbors);
            friendly_robot_id_map.emplace(friendly_robot.id(), agent_index);
        }

        for (const Robot &enemy_robot : enemy_team)
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

            Vector2 goal_position(static_cast<float>(intersection_point_set.begin()->x()),
                                  static_cast<float>(intersection_point_set.begin()->y()));
            std::size_t agent_index = addLinearVelocityRobotAgent(enemy_robot, goal_position);
            enemy_robot_id_map.emplace(enemy_robot.id(), agent_index);
        }
    }
    else
    {
        // Update Agents
        for (const Robot &friendly_robot : friendly_team)
        {
            auto agent_index_iter = friendly_robot_id_map.find(friendly_robot.id());
            if (agent_index_iter != friendly_robot_id_map.end())
            {
                // TODO: Refactor
                unsigned int agent_index = agent_index_iter->second;
                Point position = friendly_robot.position();
                agents_[agent_index]->setPosition(Vector2(position.x(), position.y()));

                if (update_world % 10 == 0)
                {
                    Vector velocity = friendly_robot.velocity();
                    agents_[agent_index]->setVelocity(Vector2(velocity.x(), velocity.y()));
                }
            }
            else
            {
                // Robot is new
            }
        }

        for (const Robot &enemy_robot : enemy_team)
        {
            auto agent_index_iter = enemy_robot_id_map.find(enemy_robot.id());
            if (agent_index_iter != enemy_robot_id_map.end())
            {
                unsigned int agent_index = agent_index_iter->second;
                Point position = enemy_robot.position();
                agents_[agent_index]->setPosition(Vector2(position.x(), position.y()));

                if (update_world % 10 == 0)
                {
                    Vector velocity = enemy_robot.velocity();
                    agents_[agent_index]->setVelocity(Vector2(velocity.x(), velocity.y()));
                }
            }
            else
            {
                // Robot is new
            }
        }
    }

    if (ball_agent_id == -1)
    {
        // Ball should be treated as an agent (obstacle)
        const Ball &ball = world.ball();
        Vector2 position(ball.position().x(), ball.position().y());
        Vector2 velocity(ball.velocity().x(), ball.velocity().y());
        Vector2 goal_pos   = position + 100 * velocity;
        float acceleration = ball.acceleration().length();
        // Minimum of 0.5-meter distance away from the ball, if the ball is an obstacle
        float ball_radius = 0.5f + BALL_AGENT_RADIUS_OFFSET;

        std::size_t agent_index = addLinearVelocityAgent(position, ball_radius, velocity, abs(velocity),
                               acceleration, addGoal(goal_pos), 0.1f);
        ball_agent_id = agent_index;
    }
    else
    {
        Point position = world.ball().position();
        agents_[ball_agent_id]->setPosition(Vector2(position.x(), position.y()));
    }

    update_world++;
}

//void HRVOSimulator::updateWorld(const World &world)
//{
//    if (update_world++ % 10 != 0) {
//        return;
//    }
//    // Reset all agents
//    agents_.clear();
//    friendly_robot_id_map.clear();
//
//    int max_neighbors =
//        std::max(1, static_cast<int>(world.friendlyTeam().getAllRobots().size()) - 1);
//    for (const Robot &friendly_robot : world.friendlyTeam().getAllRobots())
//    {
//        std::size_t agent_index = addHRVORobotAgent(friendly_robot, max_neighbors);
//        friendly_robot_id_map.emplace(friendly_robot.id(), agent_index);
//    }
//
//    for (const Robot &enemy_robot : world.enemyTeam().getAllRobots())
//    {
//        // Set goal of enemy robot to be the farthest point, when moving in the current
//        // direction
//        Segment segment(enemy_robot.position(),
//                        enemy_robot.position() + enemy_robot.velocity() * 100);
//
//        // Enemy robot should not enter the friendly defense area
//        std::unordered_set<Point> intersection_point_set =
//            intersection(world.field().friendlyDefenseArea(), segment);
//        if (intersection_point_set.empty() &&
//            contains(world.field().fieldLines(), enemy_robot.position()))
//        {
//            // If the robot is in the field, then move in the current direction
//            // towards the field edge
//            intersection_point_set = intersection(world.field().fieldLines(), segment);
//        }
//
//        if (intersection_point_set.empty())
//        {
//            // If there is no intersection point (robot is outside the field), continue
//            // moving in the current direction
//            intersection_point_set.insert(enemy_robot.position() +
//                                          enemy_robot.velocity() * 5);
//        }
//
//        Vector2 goal_position(static_cast<float>(intersection_point_set.begin()->x()),
//                              static_cast<float>(intersection_point_set.begin()->y()));
//        addLinearVelocityRobotAgent(enemy_robot, goal_position);
//    }
//
//    if (add_ball_agent)
//    {
//        // Ball should be treated as an agent (obstacle)
//        const Ball &ball = world.ball();
//        Vector2 position(ball.position().x(), ball.position().y());
//        Vector2 velocity(ball.velocity().x(), ball.velocity().y());
//        Vector2 goal_pos   = position + 100 * velocity;
//        float acceleration = ball.acceleration().length();
//        // Minimum of 0.5-meter distance away from the ball, if the ball is an obstacle
//        float ball_radius = 0.5f + BALL_AGENT_RADIUS_OFFSET;
//
//        addLinearVelocityAgent(position, ball_radius, velocity, abs(velocity),
//                               acceleration, addGoal(goal_pos), 0.1f);
//    }
//}

void HRVOSimulator::recordSimulator()
{
    const auto num_robots = static_cast<unsigned int>(getNumAgents());

    // Cache the robot radii
    std::vector<float> robot_radius(num_robots);
    for (unsigned int robot_id = 0; robot_id < num_robots; ++robot_id)
    {
        robot_radius[robot_id] = ROBOT_MAX_RADIUS_METERS;
    }

    // Initialize the previous robots position array
    std::vector<float> prev_x_pos_arr(num_robots);
    std::vector<float> prev_y_pos_arr(num_robots);
    for (int agent_id = 0; agent_id < num_robots; agent_id++)
    {
        Vector2 curr_robot_pos   = getAgentPosition(agent_id);
        prev_x_pos_arr[agent_id] = curr_robot_pos.getX();
        prev_y_pos_arr[agent_id] = curr_robot_pos.getY();
    }

    float prev_frame_time = 0.f;
    std::chrono::duration<double> computation_time(0);
    auto start_time = std::chrono::high_resolution_clock::now();        auto start_tick_time = std::chrono::high_resolution_clock::now();
    float time           = getGlobalTime();
    for (unsigned int robot_id = 0; robot_id < num_robots; robot_id++)
    {
        Vector2 curr_robot_pos = getAgentPosition(robot_id);
        float curr_robot_rad   = ROBOT_MAX_RADIUS_METERS;

        // Check for collision with other robots
        int has_collided = -1;
        for (unsigned int other_robot_id = 0; other_robot_id < num_robots;
             other_robot_id++)
        {
            Vector2 other_robot_pos = getAgentPosition(other_robot_id);
            float other_robot_rad   = ROBOT_MAX_RADIUS_METERS;
            if (robot_id != other_robot_id)
            {
                if (absSq(curr_robot_pos - other_robot_pos) <
                    std::pow(curr_robot_rad + other_robot_rad, 2.f))
                {
                    has_collided = other_robot_id;
                    break;
                }
            }
        }

        // Velocity and Speed measured in m/s
        float velocity_x = 0.f;
        float velocity_y = 0.f;
        float speed      = 0.f;
        float delta_time = time - prev_frame_time;
        // Get current robots velocity
        if (frame != 0)
        {
            float prev_x_pos = prev_x_pos_arr[robot_id];
            float prev_y_pos = prev_y_pos_arr[robot_id];

            float curr_x_pos = curr_robot_pos.getX();
            float curr_y_pos = curr_robot_pos.getY();

            velocity_x = (curr_x_pos - prev_x_pos) / delta_time;
            velocity_y = (curr_y_pos - prev_y_pos) / delta_time;
            speed      = std::hypot(velocity_x, velocity_y);

            // update previous robot x and y position
            prev_x_pos_arr[robot_id] = curr_x_pos;
            prev_y_pos_arr[robot_id] = curr_y_pos;
        }
        Vector2 goal_position =
                goals_[agents_[robot_id]->getGoalIndex()]
                        ->getCurrentGoalPosition();
        float goal_radius = agents_[robot_id]->getGoalRadius();

        output_file << frame << "," << time << ","
                    << std::to_string(computation_time.count()) << "," << robot_id
                    << "," << robot_radius[robot_id] << ","
                    << curr_robot_pos.getX() << "," << curr_robot_pos.getY()
                    << "," << velocity_x << "," << velocity_y << "," << speed
                    << "," << goal_position.getX() << "," << goal_position.getY()
                    << "," << goal_radius << "," << has_collided << ","
                    << getAgentPrefVelocity(robot_id).getX() << ","
                    << getAgentPrefVelocity(robot_id).getY()
                    << std::endl;
    }

    frame++;
}

void HRVOSimulator::updatePrimitiveSet(const TbotsProto::PrimitiveSet &primitive_set)
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
                    goal->positions_.emplace_back(
                        static_cast<float>(primitive.move().destination().x_meters()),
                        static_cast<float>(primitive.move().destination().y_meters()));
                }
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
        max_accel  = robot_constants_.robot_max_acceleration_m_per_s_2;
        max_speed  = robot_constants_.robot_max_speed_m_per_s;
        pref_speed = max_speed * PREF_SPEED_SCALE;
    }

    // TODO (#2418): Replace destination point with a list of path points
    // Get this robot's destination point, if it has a primitive
    // If this robot does not have a primitive, then set its current position as its
    // destination
    Vector2 destination_point    = position;
    float speed_at_goal          = 0.f;
    const auto &robot_primitives = primitive_set_.robot_primitives();
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
    float goal_radius        = (max_speed * timeStep_) / 2 * GOAL_RADIUS_SCALE;
    float uncertainty_offset = 0.f;

    return addHRVOAgent(position, agent_radius, velocity, max_speed, pref_speed,
                        max_accel, addGoalPositions({destination_point}, {speed_at_goal}),
                        goal_radius,
                        MAX_NEIGHBOR_SEARCH_DIST, max_neighbors, uncertainty_offset);
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
    float max_speed = robot_constants_.robot_max_speed_m_per_s;

    // Max distance which the robot can travel in one time step + scaling
    float goal_radius = (max_speed * timeStep_) / 2 * GOAL_RADIUS_SCALE;

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
    std::unique_ptr<HRVOAgent> agent = std::make_unique<HRVOAgent>(
        this, position, goal_index, neighborDist, maxNeighbors, agent_radius,
        curr_velocity, maxAccel, goalRadius, prefSpeed, maxSpeed, uncertaintyOffset);
    agents_.push_back(std::move(agent));
    return agents_.size() - 1;
}

size_t HRVOSimulator::addLinearVelocityAgent(const Vector2 &position, float agent_radius,
                                             const Vector2 &curr_velocity, float max_speed,
                                             float max_accel, size_t goal_index,
                                             float goal_radius)
{
    std::unique_ptr<LinearVelocityAgent> agent = std::make_unique<LinearVelocityAgent>(
        this, position, agent_radius, curr_velocity, max_speed, max_accel, goal_index,
        goal_radius);

    agents_.push_back(std::move(agent));
    return agents_.size() - 1;
}

std::size_t HRVOSimulator::addGoal(const Vector2 &position)
{
    std::unique_ptr<Goal> goal = std::make_unique<Goal>(position);
    goals_.push_back(std::move(goal));

    return goals_.size() - 1;
}

std::size_t HRVOSimulator::addGoalPositions(const std::vector<Vector2> &positions)
{
    std::unique_ptr<Goal> goal = std::make_unique<Goal>(positions);
    goals_.push_back(std::move(goal));

    return goals_.size() - 1;
}

std::size_t HRVOSimulator::addGoalPositions(const std::vector<Vector2> &positions,
                                            const std::vector<float> &speedAtPosition)
{
    std::unique_ptr<Goal> goal = std::make_unique<Goal>(positions, speedAtPosition);
    goals_.push_back(std::move(goal));

    return goals_.size() - 1;
}

void HRVOSimulator::doStep()
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

    if (agents_.size() == 0)
    {
        return;
    }

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

    // add if
    recordSimulator();
}

Vector HRVOSimulator::getRobotVelocity(unsigned int robot_id) const
{
    auto agent_index_iter = friendly_robot_id_map.find(robot_id);
    if (agent_index_iter != friendly_robot_id_map.end())
    {
        unsigned int agent_index  = agent_index_iter->second;
        Vector2 velocity_vector_2 = getAgentVelocity(agent_index);
        return Vector(velocity_vector_2.getX(), velocity_vector_2.getY());
    }
    // TODO: Retruning 0 vector could return optional
    //    LOG(WARNING) << "Robot_id not found for getRobotVelocity";
    std::cout << "Robot_id not found for getRobotVelocity" << std::endl;

    return Vector();
}

float HRVOSimulator::getAgentMaxAccel(std::size_t agentNo) const
{
    return agents_[agentNo]->getMaxAccel();
}

Vector2 HRVOSimulator::getAgentPosition(std::size_t agentNo) const
{
    return agents_[agentNo]->getPosition();
}

float HRVOSimulator::getAgentRadius(std::size_t agentNo) const
{
    return agents_[agentNo]->getRadius();
}

bool HRVOSimulator::hasAgentReachedGoal(std::size_t agentNo) const
{
    return agents_[agentNo]->hasReachedGoal();
}

Vector2 HRVOSimulator::getAgentVelocity(std::size_t agentNo) const
{
    return agents_[agentNo]->getVelocity();
}

Vector2 HRVOSimulator::getAgentPrefVelocity(std::size_t agentNo) const
{
    return agents_[agentNo]->getPrefVelocity();
}

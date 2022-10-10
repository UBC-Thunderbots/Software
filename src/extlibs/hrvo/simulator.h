/*
 * simulator.h
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

#pragma once

#include <fstream>
#include <limits>
#include <vector>

#include "extlibs/hrvo/agent.h"
#include "extlibs/hrvo/kd_tree.h"
#include "proto/tbots_software_msgs.pb.h"
#include "proto/visualization.pb.h"
#include "software/geom/vector.h"
#include "software/networking/threaded_proto_unix_sender.hpp"
#include "software/world/world.h"

class HRVOSimulator
{
   public:
    /**
     * Constructor
     * @param time_step The time step between each step of the simulator
     * @param robot_constants The robot constants to be used for all Agents representing a
     * robot
     * @param friendly_team_colour The colour of the friendly team
     */
    explicit HRVOSimulator(float time_step, const RobotConstants_t &robot_constants,
                           const TeamColour friendly_team_colour);

    /**
     * Reset all agents to match the state of the given world.
     * Friendly robots will use the Hybrid Reciprocal algorithm to traverse.
     * Enemy robots will go directly towards their goal without trying to avoid any
     * obstacles
     *
     * @param world The world which the simulation should be based upon
     */
    void updateWorld(const World &world);

    /**
     * Reset all friendly agents goal points to match the path of the given primitive set
     *
     * @param new_primitive_set
     */
    void updatePrimitiveSet(const TbotsProto::PrimitiveSet &new_primitive_set);

    /**
     *      Adds a new Hybrid Reciprocal Agent to the simulation based on Robot.
     *
     * @param Robot    The robot which this agent should be based on
     * @return    The index of the agent.
     */
    std::size_t addHRVORobotAgent(const Robot &robot);

    /**
     *      Adds a new Linear Velocity Agent to the simulation based on Robot.
     *
     * @param robot       The robot which this agent should be based on
     * @param destination Destination for this robot
     * @return    The index of the agent.
     */
    std::size_t addLinearVelocityRobotAgent(const Robot &robot,
                                            const Vector &destination);

    /**
     *      Adds a new agent to the simulation.
     *
     * @param position              The starting position of this agent.
     * @param agent_radius          The radius of this agent.
     * @param max_radius_inflation  The maximum amount which the radius of this agent can
     * inflate.
     * @param curr_velocity         The initial velocity of this agent.
     * @param max_speed              The maximum speed of this agent.
     * @param max_accel              The maximum acceleration of this agent.
     * @param path                  The path which this agent should take.
     * @param max_neighbor_dist  The maximum distance away which another agent can be from
     * this agent to be considered as a neighbor (i.e. velocity obstacles for it would be
     * generated).
     * @param max_neighbors  The maximum number of neighbors which this agent will
     * consider at once.
     * @return The index of the agent.
     */
    std::size_t addHRVOAgent(const Vector &position, float agent_radius,
                             float max_radius_inflation, const Vector &curr_velocity,
                             float max_speed, float max_accel, AgentPath &path,
                             float max_neighbor_dist, std::size_t max_neighbors);
    /**
     * Add a new LinearlyVelocityAgent
     * @param position              The starting position of this agent.
     * @param agent_radius          The agent_radius of this agent.
     * @param max_radius_inflation  The maximum amount which the radius of this agent can
     * inflate.
     * @param curr_velocity         The initial velocity of this agent.
     * @param max_speed             The maximum speed of this agent.
     * @param max_accel             The maximum acceleration of this agent.
     * @param goal_index            The index of the Goal which this agent should go to.
     * @param goal_radius           The goal agent_radius of this agent.
     * @return The index of the agent.
     */
    size_t addLinearVelocityAgent(const Vector &position, float agent_radius,
                                  float max_radius_inflation, const Vector &curr_velocity,
                                  float max_speed, float max_accel, AgentPath &path);

    /**
     * Performs a simulation step; updates the position, and velocity
     * of each agent, and the progress of each towards its goal by moving
     * the simulation time_step seconds forward
     */
    void doStep();

    /**
     * Get the current friendly robot velocity
     * @param robot_id The robot id of the friendly robot to retrieve velocity from
     * @return Current global velocity of robot
     */
    Vector getRobotVelocity(unsigned int robot_id) const;

    /**
     * Get friendly HRVO agent from robot id. Returns std::nullopt if agent does not exist
     * @param robot_id Robot id as represented in the real world
     * @return Optional shared pointer to an HRVO agent
     */
    std::optional<std::shared_ptr<HRVOAgent>> getFriendlyAgentFromRobotId(
        unsigned int robot_id) const;

    /**
     * Visualize this simulator to Thunderscope
     * @param robot_id The friendly robot_id which we want its velocity obstacles to be
     * visualized
     */
    void visualize(unsigned int robot_id) const;

    /**
     * Get the KDTree of Agents
     * @return KDTree of Agents
     */
    const std::unique_ptr<KdTree> &getKdTree() const;

    /**
     * Get the list of Agents in this simulator
     * @return List of Agents
     */
    const std::vector<std::shared_ptr<Agent>> &getAgents() const;

    /**
     *      Returns the maximum acceleration of a specified agent.
     *
     * @param agent_no  The number of the agent whose maximum acceleration is to be
     * retrieved.
     * @return    The present maximum acceleration of the agent.
     */
    float getAgentMaxAccel(std::size_t agent_no) const;

    /**
     *      Returns the position of a specified agent.
     *
     * @param agent_no  The number of the agent whose position is to be retrieved.
     * @return    The present position of the (center of) the agent.
     */
    Vector getAgentPosition(std::size_t agent_no) const;

    /**
     *      Returns the preferred velocity of a specified agent.
     *
     * The preferred speed of an agent is the speed it would choose
     * to take if it were not influenced by other agents.
     *
     * @param agent_no  The number of the agent whose preferred velocity is to be
     * retrieved.
     * @return    The present preferred velocity of the agent.
     */
    Vector getAgentPrefVelocity(std::size_t agent_no) const;

    /**
     *      Returns the radius of a specified agent.
     *
     * @param agent_no  The number of the agent whose radius is to be retrieved.
     * @return    The present radius of the agent.
     */
    float getAgentRadius(std::size_t agent_no) const;

    /**
     *      Returns the progress towards its goal of a specified agent.
     *
     * @param agent_no  The number of the agent whose progress towards its goal is to
     * be retrieved.
     * @return    True if the agent has reached its goal; false otherwise.
     */
    bool hasAgentReachedGoal(std::size_t agent_no) const;

    /**
     *      Returns the velocity of a specified agent.
     *
     * @param agent_no  The number of the agent whose velocity is to be retrieved.
     * @return    The present velocity of the agent.
     */
    Vector getAgentVelocity(std::size_t agent_no) const;

    /**
     *   Returns the global time of the simulation.
     *
     * @return The present global time of the simulation (zero initially).
     */
    float getGlobalTime() const
    {
        return global_time;
    }

    /**
     *   Returns the count of agents in the simulation.
     *
     * @return The count of agents in the simulation.
     */
    std::size_t getNumAgents() const
    {
        return agents.size();
    }

    /**
     *   Returns the time step of the simulation.
     *
     * @return The present time step of the simulation.
     */
    float getTimeStep() const
    {
        return time_step;
    }

    /**
     *   Returns the progress towards their goals of all agents.
     *
     * @return True if all agents have reached their goals; false otherwise.
     */
    bool haveReachedGoals() const
    {
        return reached_goals;
    }

   private:
    // PrimitiveSet which includes the path which each friendly robot should take
    TbotsProto::PrimitiveSet primitive_set;

    // Latest World which the simulator has received
    std::optional<World> world;

    // The robot constants which all agents will use
    RobotConstants_t robot_constants;

    // The global time of this hrvo simulation
    float global_time;

    // The amount of time which the simulator should advance by
    const float time_step;

    // True if all agents have reached their destination
    bool reached_goals;

    // The colour of the friendly team
    const TeamColour friendly_team_colour;

    // KdTree used to calculate the K nearest agents
    std::unique_ptr<KdTree> kd_tree;

    // List of agents (robots) in this simulation
    std::vector<std::shared_ptr<Agent>> agents;

    // robot id to agent index
    std::map<unsigned int, unsigned int> friendly_robot_id_map;
    std::map<unsigned int, unsigned int> enemy_robot_id_map;

   public:
    // The max amount (meters) which the friendly/enemy robot radius can increase by.
    // This scale is used to avoid close encounters, and reduce chance of collision.
    static constexpr float FRIENDLY_ROBOT_RADIUS_MAX_INFLATION = 0.05f;
    static constexpr float ENEMY_ROBOT_RADIUS_MAX_INFLATION    = 0.06f;

    // How much larger should the goal radius be (in meters). This is added as a safety
    // tolerance so robots do not accidentally enter the minimum distance threshold.
    // NOTE: This value must be >= 0
    static constexpr float BALL_AGENT_RADIUS_OFFSET = 0.1f;

    // The maximum distance which HRVO Agents will look for neighbors, in meters.
    // A large radius picked to allow for far visibility of neighbors so Agents have
    // enough space to decelerate and avoid collisions.
    static constexpr float MAX_NEIGHBOR_SEARCH_DIST = 2.5f;

    // The maximum number of neighbors/agents to consider when drawing velocity obstacles.
    static constexpr unsigned int MAX_NEIGHBORS = 15;

    // The max allowed difference in speed of the two robots colliding is 1.5 m/s.
    // Based on the rules, if the robot is travelling <= 0.6 m/s it will not receive a
    // penalty after a collision. To be safe, the max collision speed is set to 0.5 m/s
    static constexpr float MAX_COLLISION_SPEED = 0.5f;

    friend class Agent;
    friend class KdTree;
};

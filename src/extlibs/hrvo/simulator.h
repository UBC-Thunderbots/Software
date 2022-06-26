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
     * @param position           The starting position of this agent.
     * @param goal_index          The index of the Goal which this agent should go to.
     * @param neighborDist       The maximum neighbor distance of this agent.
     * @param maxNeighbors       The maximum neighbor count of this agent.
     * @param agent_radius       The agent_radius of this agent.
     * @param goalRadius         The goal agent_radius of this agent.
     * @param prefSpeed          The preferred speed of this agent.
     * @param maxSpeed           The maximum speed of this agent.
     * @param uncertaintyOffset  The uncertainty offset of this agent.
     * @param maxAccel           The maximum acceleration of this agent.
     * @param curr_velocity      The initial velocity of this agent.
     * @return The index of the agent.
     */
    std::size_t addHRVOAgent(const Vector &position, float agent_radius,
                             const Vector &curr_velocity, float maxSpeed, float prefSpeed,
                             float maxAccel, AgentPath &path, float neighborDist,
                             std::size_t maxNeighbors, float uncertaintyOffset);

    /**
     * Add a new LinearlyVelocityAgent
     * @param position      The starting position of this agent.
     * @param agent_radius  The agent_radius of this agent.
     * @param curr_velocity The initial velocity of this agent.
     * @param max_speed     The maximum speed of this agent.
     * @param max_accel     The maximum acceleration of this agent.
     * @param goal_index     The index of the Goal which this agent should go to.
     * @param goal_radius   The goal agent_radius of this agent.
     * @return The index of the agent.
     */
    size_t addLinearVelocityAgent(const Vector &position, float agent_radius,
                                  const Vector &curr_velocity, float max_speed,
                                  float max_accel, AgentPath &path);

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
     * Update the local velocity of the Agent which represents robot with robot_id
     *
     * @param robot_id The robot id of the friendly robot to update velocity of
     * @param new_velocity New velocity of the friendly robot
     */
    void updateFriendlyRobotVelocity(const RobotId robot_id, const Vector &new_velocity) const;

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
     *   Returns the time step of the simulation.
     *
     * @return The present time step of the simulation.
     */
    float getTimeStep() const
    {
        return time_step;
    }

   private:
    // PrimitiveSet which includes the path which each friendly robot should take
    TbotsProto::PrimitiveSet primitive_set;

    // True if the ball should be treated as an agent (obstacle)
    // NOTE: This will take effect the next time we receive a world, and we know
    //       the current ball position and velocity
    bool add_ball_agent;
    std::size_t ball_agent_id;

    // The robot constants which all agents will use
    RobotConstants_t robot_constants;

    // The global time of this hrvo simulation
    float global_time;

    // The amount of time which the simulator should advance by
    const float time_step;

    // The last time which the velocity of the robot was updated
    float last_time_velocity_updated;

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
    // The scale which friendly robots should be larger than friendly robots
    // This scale is used to avoid close encounters, and reduce chance of collision
    static constexpr float FRIENDLY_ROBOT_RADIUS_SCALE = 1.25f;

    // The scale which enemy robots should be larger than their actual size
    // This scale is used to avoid close encounters, and reduce chance of collision
    static constexpr float ENEMY_ROBOT_RADIUS_SCALE = 1.5f;

    // How much larger should the goal radius be. This is added as a safety tolerance so
    // robots do not "teleport" over the goal between simulation frames.
    static constexpr float GOAL_RADIUS_SCALE = 1.05f;

    // How much larger should the goal radius be (in meters). This is added as a safety
    // tolerance so robots do not accidentally enter the minimum distance threshold.
    // NOTE: This value must be >= 0
    static constexpr float BALL_AGENT_RADIUS_OFFSET = 0.1f;

    // The scale multiple of max robot speed which the preferred speed will be set at.
    // pref_speed = max_speed * PREF_SPEED_SCALE
    // NOTE: This scale multiple must be <= 1
    static constexpr float PREF_SPEED_SCALE = 0.85f;

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
    friend class Goal;
    friend class KdTree;
};

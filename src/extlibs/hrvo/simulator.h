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

#include <limits>
#include <vector>

#include "extlibs/hrvo/agent.h"
#include "extlibs/hrvo/goal.h"
#include "extlibs/hrvo/kd_tree.h"
#include "extlibs/hrvo/vector2.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/world/world.h"

class Simulator
{
   public:
    explicit Simulator(float time_step);
    ~Simulator() = default;

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
     * @param primitive_set
     */
    void updatePrimitiveSet(const TbotsProto::PrimitiveSet &primitive_set);

    /**
     *      Adds a new Hybrid Reciprocal Agent to the simulation based on Robot.
     *
     * @param Robot    The robot which this agent should be based on
     * @param velocity The number of neighboring agents which the HRVO algorithm should
     * consider when calculating new velocity
     * @return    The index of the agent.
     */
    std::size_t addHRVORobotAgent(const Robot &robot, int max_neighbors = 10);

    /**
     *      Adds a new Linear Velocity Agent to the simulation based on Robot.
     *
     * @param robot       The robot which this agent should be based on
     * @param destination Destination for this robot
     * @return    The index of the agent.
     */
    std::size_t addLinearVelocityRobotAgent(const Robot &robot,
                                            const Vector2 &destination);

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
    std::size_t addHRVOAgent(const Vector2 &position, float agent_radius,
                             const Vector2 &curr_velocity, float maxSpeed,
                             float prefSpeed, float maxAccel, std::size_t goal_index,
                             float goalRadius, float neighborDist,
                             std::size_t maxNeighbors, float uncertaintyOffset);

    /**
     *
     * @param position      The starting position of this agent.
     * @param agent_radius  The agent_radius of this agent.
     * @param curr_velocity The initial velocity of this agent.
     * @param max_speed     The maximum speed of this agent.
     * @param max_accel     The maximum acceleration of this agent.
     * @param goal_index     The index of the Goal which this agent should go to.
     * @param goal_radius   The goal agent_radius of this agent.
     * @return The index of the agent.
     */
    size_t addLinearVelocityAgent(const Vector2 &position, float agent_radius,
                                  const Vector2 &curr_velocity, float max_speed,
                                  float max_accel, size_t goal_index, float goal_radius);

    // TODO (#2373): Remove goals_ list when goal is a part of Agent
    /**
     *      Adds a new goal to the simulation.
     *
     * @param position  The position of this goal.
     * @return    The number of the goal.
     */
    std::size_t addGoal(const Vector2 &position);
    std::size_t addGoalPositions(const std::vector<Vector2> &positions);
    std::size_t addGoalPositions(const std::vector<Vector2> &positions,
                                 const std::vector<float> &speedAtPosition);

    /**
     * Performs a simulation step; updates the position, and velocity
     * of each agent, and the progress of each towards its goal by moving
     * the simulation time_step seconds forward
     */
    void doStep();

    /**
     *      Returns the maximum acceleration of a specified agent.
     *
     * @param agentNo  The number of the agent whose maximum acceleration is to be
     * retrieved.
     * @return    The present maximum acceleration of the agent.
     */
    float getAgentMaxAccel(std::size_t agentNo) const;

    /**
     *      Returns the position of a specified agent.
     *
     * @param agentNo  The number of the agent whose position is to be retrieved.
     * @return    The present position of the (center of) the agent.
     */
    Vector2 getAgentPosition(std::size_t agentNo) const;

    /**
     *      Returns the preferred velocity of a specified agent.
     *
     * The preferred speed of an agent is the speed it would choose
     * to take if it were not influenced by other agents.
     *
     * @param agentNo  The number of the agent whose preferred velocity is to be
     * retrieved.
     * @return    The present preferred velocity of the agent.
     */
    Vector2 getAgentPrefVelocity(std::size_t agentNo) const;

    /**
     *      Returns the radius of a specified agent.
     *
     * @param agentNo  The number of the agent whose radius is to be retrieved.
     * @return    The present radius of the agent.
     */
    float getAgentRadius(std::size_t agentNo) const;

    /**
     *      Returns the progress towards its goal of a specified agent.
     *
     * @param agentNo  The number of the agent whose progress towards its goal is to
     * be retrieved.
     * @return    True if the agent has reached its goal; false otherwise.
     */
    bool hasAgentReachedGoal(std::size_t agentNo) const;

    /**
     *      Returns the velocity of a specified agent.
     *
     * @param agentNo  The number of the agent whose velocity is to be retrieved.
     * @return    The present velocity of the agent.
     */
    Vector2 getAgentVelocity(std::size_t agentNo) const;

    /**
     *   Returns the global time of the simulation.
     *
     * @return The present global time of the simulation (zero initially).
     */
    float getGlobalTime() const
    {
        return globalTime_;
    }

    /**
     *   Returns the count of agents in the simulation.
     *
     * @return The count of agents in the simulation.
     */
    std::size_t getNumAgents() const
    {
        return agents_.size();
    }

    /**
     *   Returns the count of goals in the simulation.
     *
     * @return The count of goals in the simulation.
     */
    std::size_t getNumGoals() const
    {
        return goals_.size();
    }

    /**
     *   Returns the time step of the simulation.
     *
     * @return The present time step of the simulation.
     */
    float getTimeStep() const
    {
        return timeStep_;
    }

    /**
     *   Returns the progress towards their goals of all agents.
     *
     * @return True if all agents have reached their goals; false otherwise.
     */
    bool haveReachedGoals() const
    {
        return reachedGoals_;
    }

   public:
    // KdTree used to calculate the K nearest agents
    std::unique_ptr<KdTree> kdTree_;

    // The global time of this hrvo simulation
    float globalTime_;

    // The amount of time which the simulator should advance by
    float timeStep_;

    // True if all agents have reached their destination
    bool reachedGoals_;

    // List of agents (robots) in this simulation
    std::vector<std::unique_ptr<Agent>> agents_;
    // TODO (#2373): Remove goals_ list when goal is a part of Agent
    std::vector<std::unique_ptr<Goal>> goals_;

   private:
    // friendly robot id to agent index
    std::map<unsigned int, unsigned int> friendly_robot_id_map;

    // PrimitiveSet which includes the path which each friendly robot should take
    TbotsProto::PrimitiveSet primitive_set_;

    // True if the ball should be treated as an agent (obstacle)
    // NOTE: This will take effect the next time we receive a world, and we know
    //       the current ball position and velocity
    bool add_ball_agent = false;

    // The scale which friendly robots should be larger than friendly robots
    // This scale is used to avoid close encounters, and reduce chance of collision
    const float friendly_robot_radius_scale = 1.25f;

    // The scale which enemy robots should be larger than their actual size
    // This scale is used to avoid close encounters, and reduce chance of collision
    const float enemy_robot_radius_scale = 2.f;

    // How much larger should the goal radius be. This is added as a safety tolerance so
    // robots do not "teleport" over the goal between simulation frames.
    const float goal_radius_scale = 1.05f;

    // How much larger should the goal radius be (in meters). This is added as a safety
    // tolerance so robots do not accidentally enter the minimum distance threshold.
    // NOTE: This value must be >= 0
    const float ball_agent_radius_offset = 0.1f;

    // The scale multiple of max robot speed which the preferred speed will be set at.
    // pref_speed = max_speed * pref_speed_scale
    // NOTE: This scale multiple must be <= 1
    const float pref_speed_scale = 0.75f;

    // The maximum distance which HRVO Agents will look for neighbors, in meters.
    // A large radius picked to allow for far visibility of neighbors so Agents have
    // enough space to decelerate and avoid collisions.
    const float max_neighbor_search_dist = 7.f;

    friend class Agent;
    friend class Goal;
    friend class KdTree;
};

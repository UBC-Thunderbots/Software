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

    void updateWorld(const World &world);

    void updatePrimitiveSet(const TbotsProto::PrimitiveSet &primitive_set);

    /**
     *      Adds a new agent to the simulation based on Robot.
     *
     * @param Robot    The robot which this agent should be based on
     * @param velocity The number of neighboring agents which the HRVO algorithm should
     * consider when calculating new velocity
     * @return    The number of the agent.
     */
    std::size_t addHRVORobotAgent(const Robot &robot, int max_neighbors = 10);

    std::size_t addLinearVelocityRobotAgent(const Robot &robot,
                                            const Vector2 &destination);

    /**
     *      Adds a new agent to the simulation.
     *
     * @param position           The starting position of this agent.
     * @param goalNo             The goal number of this agent.
     * @param neighborDist       The maximum neighbor distance of this agent.
     * @param maxNeighbors       The maximum neighbor count of this agent.
     * @param radius             The radius of this agent.
     * @param goalRadius         The goal radius of this agent.
     * @param prefSpeed          The preferred speed of this agent.
     * @param maxSpeed           The maximum speed of this agent.
     * @param uncertaintyOffset  The uncertainty offset of this agent.
     * @param maxAccel           The maximum acceleration of this agent.
     * @param velocity           The initial velocity of this agent.
     * @return    The number of the agent.
     */
    std::size_t addHRVOAgent(const Vector2 &position, std::size_t goalNo,
                             float neighborDist, std::size_t maxNeighbors, float radius,
                             float goalRadius, float prefSpeed, float maxSpeed,
                             float uncertaintyOffset, float maxAccel,
                             const Vector2 &velocity);

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
     *  Performs a simulation step; updates the orientation, position, and velocity
     * of each agent, and the progress of each towards its goal.
     */
    void doStep();

    /**
     *      Returns the goal number of a specified agent.
     *
     * @param agentNo  The number of the agent whose goal number is to be retrieved.
     * @return    The present goal number of the agent.
     */
    std::size_t getAgentGoal(std::size_t agentNo) const;

    /**
     *      Returns the goal radius of a specified agent.
     *
     * @param agentNo  The number of the agent whose goal radius is to be retrieved.
     * @return    The present goal radius of the agent
     */
    float getAgentGoalRadius(std::size_t agentNo) const;

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
    bool getAgentReachedGoal(std::size_t agentNo) const;

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
     *      Returns the position of a specified goal.
     *
     * @param goalNo  The number of the goal whose position is to be retrieved.
     * @return    The position of the goal.
     */
    Vector2 getGoalPosition(std::size_t goalNo) const;

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

    /**
     *      Sets the goal number of a specified agent.
     *
     * @param agentNo  The number of the agent whose goal number is to be modified.
     * @param goalNo   The replacement goal number.
     */
    void setAgentGoal(std::size_t agentNo, std::size_t goalNo);

    void setAgentGoalPosition(size_t agentNo, Vector2 position);

    /**
     *      Sets the goal radius of a specified agent.
     *
     * @param agentNo     The number of the agent whose goal radius is to be modified.
     * @param goalRadius  The replacement goal radius.
     */
    void setAgentGoalRadius(std::size_t agentNo, float goalRadius);

    /**
     *      Sets the maximum linear acceleraton of a specified agent.
     *
     * @param agentNo   The number of the agent whose maximum acceleration is to be
     * modified. @param maxAccel  The replacement maximum acceleration.
     */
    void setAgentMaxAccel(std::size_t agentNo, float maxAccel);

    /**
     *      Sets the maximum speed of a specified agent.
     *
     * @param agentNo   The number of the agent whose maximum speed is to be modified.
     * @param maxSpeed  The replacement maximum speed.
     */
    void setAgentMaxSpeed(std::size_t agentNo, float maxSpeed);

    /**
     *      Sets the position of a specified agent.
     * @param agentNo   The number of the agent whose position is to be modified.
     * @param position  The replacement position.
     */
    void setAgentPosition(std::size_t agentNo, const Vector2 &position);

    /**
     *      Sets the radius of a specified agent.
     *
     * @param agentNo  The number of the agent whose radius is to be modified.
     * @param radius   The replacement radius.
     */
    void setAgentRadius(std::size_t agentNo, float radius);

    /**
     *      Sets the velocity of a specified agent.
     *
     * @param agentNo   The number of the agent whose velocity is to be modified.
     * @param velocity  The replacement velocity.
     */
    void setAgentVelocity(std::size_t agentNo, const Vector2 &velocity);

    /**
     *      Sets the time step of the simulation.
     *
     * @param timeStep  The replacement time step of the simulation.
     */
    void setTimeStep(float timeStep)
    {
        timeStep_ = timeStep;
    }

    //	private:
   public:
    Simulator(const Simulator &other);
    Simulator &operator=(const Simulator &other);


    std::unique_ptr<KdTree> kdTree_;
    float globalTime_;
    float timeStep_;
    bool reachedGoals_;
    std::vector<std::unique_ptr<Agent>> agents_;
    std::vector<std::unique_ptr<Goal>>
        goals_;  // won't be one to one since not all agents will have a goal

    TbotsProto::PrimitiveSet primitive_set_;
    // robot id to agent index
    std::map<unsigned int, unsigned int> friendly_robot_id_map;

    friend class Agent;
    friend class Goal;
    friend class KdTree;
};

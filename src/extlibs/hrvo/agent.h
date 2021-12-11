/*
 * agent.h
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

#include <cstddef>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "goal.h"
#include "simulator.h"
#include "vector2.h"

/**
 * An agent/robot in the HRVO simulation.
 */
class Agent
{
   private:
    /**
     * A candidate point.
     */
    class Candidate
    {
       public:
        Candidate() : velocityObstacle1_(0), velocityObstacle2_(0) {}

        // The position of the candidate point.
        Vector2 position_;

        // The number of the first velocity obstacle.
        int velocityObstacle1_;

        // The number of the second velocity obstacle.
        int velocityObstacle2_;
    };

    /**
     * A hybrid reciprocal velocity obstacle.
     */
    class VelocityObstacle
    {
       public:
        VelocityObstacle() {}

        // The position of the apex of the hybrid reciprocal velocity obstacle.
        Vector2 apex_;

        // The direction of the first side of the hybrid reciprocal velocity obstacle.
        Vector2 side1_;

        // The direction of the second side of the hybrid reciprocal velocity obstacle.
        Vector2 side2_;
    };


    /**
     * Constructor
     *
     * @param simulator  The simulation which the Agent is a part of
     */
    explicit Agent(Simulator *simulator);

    /**
     * Constructor
     *
     * @param simulator  The simulation.
     * @param position   The starting position of this agent.
     * @param goalNo     The goal number of this agent.
     */
    Agent(Simulator *simulator, const Vector2 &position, std::size_t goalNo);

    /**
     * Constructor
     *
     * @param simulator          The simulation.
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
     * @param orientation        The initial orientation (in radians) of this agent.
     */
    Agent(Simulator *simulator, const Vector2 &position, std::size_t goalNo,
          float neighborDist, std::size_t maxNeighbors, float radius,
          const Vector2 &velocity, float maxAccel, float goalRadius, float prefSpeed,
          float maxSpeed, float orientation, float uncertaintyOffset);

    /**
     * Computes the neighbors of this agent.
     */
    void computeNeighbors();

    /**
     * Computes the new velocity of this agent.
     */
    void computeNewVelocity();

    /**
     * Computes the preferred velocity of this agent.
     */
    void computePreferredVelocity();

    /**
     * Get the current Agent velocity
     */
    Vector2 getVelocity() const;

    /**
     * Inserts a neighbor into the set of neighbors of this agent.
     *
     * @param  agentNo  The number of the agent to be inserted.
     * @param  rangeSq  The squared range around this agent.
     */
    void insertNeighbor(std::size_t agentNo, float &rangeSq);

    /**
     * Updates the orientation, position, and velocity of this agent.
     */
    void update();

   public:  // A
    Simulator *const simulator_;
    Vector2 newVelocity_;
    Vector2 position_;
    Vector2 prefVelocity_;
    Vector2 velocity_;
    std::size_t goalNo_;
    std::size_t maxNeighbors_;
    float goalRadius_;
    float maxAccel_;
    float maxSpeed_;
    float neighborDist_;
    float orientation_;
    float prefSpeed_;
    float radius_;
    float uncertaintyOffset_;
    bool reachedGoal_;
    std::multimap<float, Candidate> candidates_;
    std::set<std::pair<float, std::size_t>> neighbors_;
    std::vector<VelocityObstacle> velocityObstacles_;

    friend class KdTree;
    friend class Simulator;
};

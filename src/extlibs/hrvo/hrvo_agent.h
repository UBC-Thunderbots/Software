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

#include "agent.h"
#include "goal.h"
#include "simulator.h"
#include "vector2.h"

/**
 * An agent/robot in the simulation which uses the HRVO algorithm.
 */
class HRVOAgent : public Agent
{
   public:
    /**
     * Constructor
     *
     * @param simulator  The simulation which the Agent is a part of
     */
    explicit HRVOAgent(Simulator *simulator);

    /**
     * Constructor
     *
     * @param simulator  The simulation.
     * @param position   The starting position of this agent.
     * @param goalNo     The goal number of this agent.
     */
    HRVOAgent(Simulator *simulator, const Vector2 &position, std::size_t goalNo);

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
     */
    HRVOAgent(Simulator *simulator, const Vector2 &position, std::size_t goalNo,
              float neighborDist, std::size_t maxNeighbors, float radius,
              const Vector2 &velocity, float maxAccel, float goalRadius, float prefSpeed,
              float maxSpeed, float uncertaintyOffset);

    ~HRVOAgent() override = default;

    /**
     * Computes the new velocity of this agent.
     */
    void computeNewVelocity() override;

    /**
     * Computes the neighbors of this agent.
     */
    void computeNeighbors();

    /**
     * Computes the preferred velocity of this agent.
     */
    void computePreferredVelocity();

    /**
     * Inserts a neighbor into the set of neighbors of this agent.
     *
     * @param  agentNo  The number of the agent to be inserted.
     * @param  rangeSq  The squared range around this agent.
     */
    void insertNeighbor(std::size_t agentNo, float &rangeSq);


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

   public:  // A
    float prefSpeed_;
    Vector2 prefVelocity_;
    std::size_t maxNeighbors_;
    float neighborDist_;
    float uncertaintyOffset_;
    std::multimap<float, Candidate> candidates_;
    // distance -> Agent Index
    std::set<std::pair<float, std::size_t>> neighbors_;
    std::vector<VelocityObstacle> velocityObstacles_;

    friend class KdTree;
    friend class Simulator;
};

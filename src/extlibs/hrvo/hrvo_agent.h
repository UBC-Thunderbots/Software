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
#include "simulator.h"
#include "software/geom/vector.h"

/**
 * An agent/robot in the simulation which uses the HRVO algorithm to motion plan towards
 * the destination while avoiding obstacles.
 */
class HRVOAgent : public Agent
{
   public:
    /**
     * Constructor
     *
     * @param simulator          The simulation.
     * @param position           The starting position of this agent.
     * @param goalIndex          The goal number of this agent.
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
    HRVOAgent(HRVOSimulator *simulator, const Vector &position, float neighborDist,
              std::size_t maxNeighbors, float radius, const Vector &velocity,
              float maxAccel, AgentPath &path, float prefSpeed, float maxSpeed,
              float uncertaintyOffset);

    /**
     * Computes the new velocity of this agent.
     */
    void computeNewVelocity() override;

    /**
     * Create the hybrid reciprocal velocity obstacle which other_agent should see for
     * this Agent
     *
     * @param other_agent The Agent which this hybrid reciprocal velocity obstacle is
     * being generated for
     * @return The hybrid reciprocal velocity obstacle which other_agent should see for
     * this Agent
     */
    VelocityObstacle createVelocityObstacle(const Agent &other_agent) override;

    /**
     * Computes the maxNeighbors nearest neighbors of this agent.
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

    /**
     * Get a list of circles which represent the new velocity candidates
     * @param circle_rad The radius of the circles which represent candidates
     * @return list of circles which represent the new velocity candidates
     */
    std::vector<Circle> getCandidateVelocitiesAsCircles(
        const float circle_rad = 0.03f) const;

    /**
     * Get a list of triangles (polygons) which represent the velocity obstacles
     * which this HRVO Agent currently sees.
     * @return A list of polygons which represent velocity obstacles
     */
    std::vector<TbotsProto::VelocityObstacle> getVelocityObstaclesAsProto() const;

    /**
     * Update preferred speed of Agent. The preferred speed represents the speed which we
     * would like this Agent to travel at.
     * @param new_pref_speed New preferred speed
     */
    void setPreferredSpeed(float new_pref_speed);

   private:
    /**
     * A candidate point.
     */
    class Candidate
    {
       public:
        Candidate() : velocityObstacle1_(0), velocityObstacle2_(0) {}

        // The position of the candidate point.
        Vector position_;

        // The number of the first velocity obstacle.
        int velocityObstacle1_;

        // The number of the second velocity obstacle.
        int velocityObstacle2_;
    };

   public:
    float prefSpeed_;

    std::size_t maxNeighbors_;
    float neighborDist_;
    float uncertaintyOffset_;
    std::multimap<float, Candidate> candidates_;
    // distance -> Agent Index
    std::set<std::pair<float, std::size_t>> neighbors_;
    std::vector<VelocityObstacle> velocityObstacles_;

    // TODO (#2519): Remove magic numbers
    // Increasing deceleration distance to reduce the chance of overshooting the
    // destination
    static constexpr float decel_dist_multiplier = 1.2f;
    // Decreasing preferred speed during deceleration to reduce the chance of
    // overshooting the destination
    static constexpr float decel_pref_speed_multiplier = 0.6f;

    friend class KdTree;
    friend class Simulator;
};

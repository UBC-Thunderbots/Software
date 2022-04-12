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
     * @param velocity           The initial velocity of this agent.
     * @param pref_speed          The preferred speed of this agent.
     * @param max_speed           The maximum speed of this agent.
     * @param max_accel           The maximum acceleration of this agent.
     * @param path           The path which this agent should follow.
     * @param radius             The radius of this agent.
     * @param max_num_neighbors       The maximum neighbor count of this agent.
     * @param max_neighbor_dist       The maximum neighbor distance of this agent.
     * @param uncertainty_offset  The uncertainty offset of this agent.
     */
    HRVOAgent(HRVOSimulator *simulator, const Vector &position, const Vector &velocity,
              float pref_speed, float max_speed, float max_accel, AgentPath &path,
              float radius, std::size_t max_num_neighbors, float max_neighbor_dist,
              float uncertainty_offset);

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
    Agent::VelocityObstacle createVelocityObstacle(const Agent &other_agent) override;

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
     * @param  agent_id  The id of the agent to be inserted.
     * @param  range_squared  The squared range around this agent.
     */
    void insertNeighbor(std::size_t agent_id, float range_squared);

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
    std::vector<Polygon> getVelocityObstaclesAsPolygons() const;

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
        Candidate() : velocity_obstacle1_(0), velocity_obstacle2_(0) {}

        // The position of the candidate point.
        Vector position_;

        // The number of the first velocity obstacle.
        int velocity_obstacle1_;

        // The number of the second velocity obstacle.
        int velocity_obstacle2_;
    };

   public:
    float pref_speed_;

    std::size_t max_neighbors_;
    float neighbor_dist_;
    float uncertainty_offset_;
    std::multimap<float, Candidate> candidates_;
    // distance -> Agent Index
    std::set<std::pair<float, std::size_t>> neighbors_;
    std::vector<VelocityObstacle> velocity_obstacles_;

    friend class KdTree;
    friend class Simulator;
};

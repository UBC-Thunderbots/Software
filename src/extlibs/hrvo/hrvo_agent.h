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
#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
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
     * @param simulator             The simulation.
     * @param position              The starting position of this agent.
     * @param max_neighbor_dist     The maximum distance away which another agent can be
     * from this agent to be considered as a neighbor (i.e. velocity obstacles for it
     * would be created)
     * @param max_neighbors         The maximum number of other agents which this agent
     * will try to avoid collisions with at a time.
     * @param radius                The radius of this agent.
     * @param max_radius_inflation  The maximum amount which the radius of this agent can
     * inflate.
     * @param velocity              The initial velocity of this agent.
     * @param max_accel              The maximum acceleration of this agent.
     * @param path                  The path which this agent should take.
     * @param max_speed              The maximum speed of this agent.
     */
    HRVOAgent(HRVOSimulator *simulator, const Vector &position, float max_neighbor_dist,
              std::size_t max_neighbors, float radius, float max_radius_inflation,
              const Vector &velocity, float max_accel, AgentPath &path, float max_speed);

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
     * @param neighbor_dist_threshold The max distance away which another agent can be to
     * be considered a neighbor.
     * Computes the `maxNeighbors` nearest neighbors of this agent which are within
     * `neighbor_dist_threshold`.
     */
    void computeNeighbors(double neighbor_dist_threshold);

    /**
     * Computes the preferred velocity of this agent.
     */
    void computePreferredVelocity();

    /**
     * Inserts a neighbor into the set of neighbors of this agent.
     *
     * @param  agent_no  The number of the agent to be inserted.
     * @param  range_sq  The squared range around this agent.
     */
    void insertNeighbor(std::size_t agent_no, float &range_sq);

    /**
     * Update the primitive which this agent is currently pursuing.
     *
     * @param new_primitive The new primitive to pursue
     * @param world The world in which the new primitive is being pursued
     */
    void updatePrimitive(const TbotsProto::Primitive &new_primitive, const World &world);

    /**
     * Get a list of circles which represent the new velocity candidates
     * @param circle_rad The radius of the circles which represent candidates
     * @return list of circles which represent the new velocity candidates
     */
    std::vector<Circle> getCandidateVelocitiesAsCircles(
        const float circle_rad = 0.03f) const;

    /**
     * Get a list of velocity obstacle protos which represent the velocity obstacles
     * in position space (i.e. the velocity obstacles are shifted by the agents position)
     * @return A list of velocity obstacles protos which represent velocity obstacles in
     * position space (as opposed to velocity space)
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
     * A candidate point is a internal structure used when computing new velocities. It is
     * composed of a potential new velocity and the index of two VelocityObstacles in
     * velocity_obstacles_ that were used to compute it.
     */
    class Candidate
    {
       public:
        Candidate() : velocity_obstacle_1_(0), velocity_obstacle_2_(0) {}

        // The velocity of the candidate.
        Vector velocity;

        // The index of the first VelocityObstacle in velocity_obstacles_ used to compute
        // this candidate.
        int velocity_obstacle_1_;

        // The index of the second VelocityObstacle in velocity_obstacles_ used to compute
        // this candidate.
        int velocity_obstacle_2_;
    };

    // Percentage of preferred speed that we accept as the lower bound of a potential new
    // velocity
    static constexpr float MIN_PREF_SPEED_MULTIPLER = 0.5f;

    /**
     * Returns the first velocity obstacle intersected by the Candidate point in
     * velocity_obstacles_
     *
     * @param candidate the candidate point to check against all velocity obstacles in
     * velocity_obstacles_
     *
     * @return the index of the first velocity obstacle that the given candidate point
     * intersects in velocity_obstacles_, or std::nullopt if the candidate does not
     * intersect any
     */
    std::optional<int> findIntersectingVelocityObstacle(const Candidate &candidate) const;

    /**
     * Returns true if the given candidate point doesn't intersct any obstacle in
     * velocity_obstacles_ and is faster than the minimum preferred speed.
     *
     * @param candidate	the candidate point to consider
     *
     * @return true if it the candidate point doesn't intersect any obstacle and is fast
     */
    bool isIdealCandidate(const Candidate &candidate) const;

    /**
     * Returns true if the given candidate point is faster than the minimum preferred
     * speed.
     *
     * @param candidate the candidate point to consider
     *
     * @return true if the candidate is faster or equal to the minimum preferred speed,
     * false otherwise
     */
    bool isCandidateFast(const Candidate &candidate) const;

    /**
     * Returns true if the candidate point is slower than the minimum preferred speed.
     *
     * @param candidate the candidate point to consider
     *
     * @return true if the candidate is slower than the minimum preferred speed, false if
     * the speed is faster or the equal to the minimum preferred speed.
     */
    bool isCandidateSlow(const Candidate &candidate) const;

    /**
     * Returns true if the candidate is faster than the current new_velocity_.
     *
     * @param candidate the candidate point to consider
     *
     * @return true if the candidate point is faster than new_velocity_, false if the
     * candidate is as fast or slower
     */
    bool isCandidateFasterThanCurrentSpeed(const Candidate &candidate) const;

    /**
     * Compute all the velocity obstacles that this Agent should take into account and
     * add it to `velocityObstacles_`.
     */
    void computeVelocityObstacles();

   public:
    float pref_speed_;

    std::size_t max_neighbors_;
    float max_neighbor_dist;
    float uncertainty_offset_;
    std::multimap<float, Candidate> candidates_;
    // distance -> Agent Index
    std::set<std::pair<float, std::size_t>> neighbors_;
    std::vector<VelocityObstacle> velocity_obstacles_;
    std::vector<ObstaclePtr> static_obstacles;
    std::optional<ObstaclePtr> ball_obstacle;
    RobotNavigationObstacleFactory obstacle_factory;

    // TODO (#2519): Remove magic numbers
    // Increasing deceleration distance to reduce the chance of overshooting the
    // destination
    static constexpr float decel_dist_multiplier = 1.2f;
    // Decreasing preferred speed during deceleration to reduce the chance of
    // overshooting the destination
    static constexpr float decel_pref_speed_multiplier = 0.6f;

    // The scale multiple of max robot speed which the preferred speed will be set at.
    // pref_speed = max_speed * PREF_SPEED_SCALE
    // NOTE: This scale multiple must be <= 1
    static constexpr float PREF_SPEED_SCALE = 0.85f;

    friend class KdTree;
    friend class Simulator;
};

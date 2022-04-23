/*
 * agent.cpp
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

#include "hrvo_agent.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include "kd_tree.h"
#include "path.h"
#include "software/geom/vector.h"

HRVOAgent::HRVOAgent(HRVOSimulator *simulator, const Vector &position, float neighborDist,
                     std::size_t maxNeighbors, float radius, const Vector &velocity,
                     float maxAccel, AgentPath &path, float prefSpeed, float maxSpeed,
                     float uncertaintyOffset)
    : Agent(simulator, position, radius, velocity, velocity, maxSpeed, maxAccel, path),
      maxNeighbors_(maxNeighbors),
      neighborDist_(neighborDist),
      prefSpeed_(prefSpeed),
      uncertaintyOffset_(uncertaintyOffset)
{
}

void HRVOAgent::computeNeighbors()
{
    neighbors_.clear();

    if (path.getCurrentPathPoint() == std::nullopt)
    {
        return;
    }

    Vector current_dest = path.getCurrentPathPoint().value().getPosition();

    float new_neighbor_dist =
        std::min(static_cast<double>(neighborDist_),
                 (position_ - current_dest).length() + path.getPathRadius());

    simulator_->getKdTree()->query(this, new_neighbor_dist);
}

VelocityObstacle HRVOAgent::createVelocityObstacle(const Agent &other_agent)
{
    VelocityObstacle velocityObstacle;
    if ((position_ - other_agent.getPosition()).lengthSquared() >
        std::pow(radius_ + other_agent.getRadius(), 2))
    {
        // This Agent is not colliding with other agent
        const float angle =
            (position_ - other_agent.getPosition()).orientation().toRadians();

        // The opening angle of the velocity obstacle
        // opening angle = arcsin((rad_A + rad_B) / distance_BA)
        const float openingAngle =
            std::asin((radius_ + other_agent.getRadius()) /
                      (position_ - other_agent.getPosition()).length());
        // Direction of the two edges of the velocity obstacles
        velocityObstacle.right_side =
            Vector::createFromAngle(Angle::fromRadians(angle - openingAngle));
        velocityObstacle.left_side =
            Vector::createFromAngle(Angle::fromRadians(angle + openingAngle));

        const float d = std::sin(2.f * openingAngle);

        // This shifts one side of the velocity obstacle to share the responsibility
        // of avoiding collision with other agent. This assumes that other agent will also
        // be running HRVO
        if ((position_ - other_agent.getPosition())
                .determinant(other_agent.getPrefVelocity() - pref_velocity_) > 0.0f)
        {
            // Relative velocity is in the right half of velocity obstacle (VO)
            // Shift the VO apex to the left so the right side is smaller, making the
            // VO a Hybrid Reciprocal Velocity Obstacle (HRVO)
            const float s = 0.5f *
                            (other_agent.getVelocity() - velocity_)
                                .determinant(velocityObstacle.left_side) /
                            d;

            velocityObstacle.apex_ =
                velocity_ + s * velocityObstacle.right_side -
                (position_ - other_agent.getPosition())
                    .normalize((uncertaintyOffset_ *
                                (position_ - other_agent.getPosition()).length() /
                                (radius_ + other_agent.getRadius())));
        }
        else
        {
            // Relative velocity is in the left half of velocity obstacle (VO)
            // Shift the VO apex to the right so the left side is smaller, making the
            // VO a Hybrid Reciprocal Velocity Obstacle (HRVO)
            const float s = 0.5f *
                            (other_agent.getVelocity() - velocity_)
                                .determinant(velocityObstacle.right_side) /
                            d;

            velocityObstacle.apex_ =
                velocity_ + s * velocityObstacle.left_side -
                (position_ - other_agent.getPosition())
                    .normalize(uncertaintyOffset_ *
                               (position_ - other_agent.getPosition()).length() /
                               (other_agent.getRadius() + radius_));
        }
    }
    else
    {
        // This Agent is colliding with other agent
        // Uses Reciprocal Velocity Obstacle (RVO) with the sides being 180 degrees
        // apart from each other
        velocityObstacle.apex_ =
            0.5f * (other_agent.getVelocity() + velocity_) -
            (position_ - other_agent.getPosition())
                .normalize(uncertaintyOffset_ +
                           0.5f *
                               (other_agent.getRadius() + radius_ -
                                (position_ - other_agent.getPosition()).length()) /
                               simulator_->getTimeStep());
        velocityObstacle.right_side =
            (other_agent.getPosition() - position_).perpendicular().normalize();
        velocityObstacle.left_side = -velocityObstacle.right_side;
    }

    return velocityObstacle;
}

void HRVOAgent::computeNewVelocity()
{
    // Based on The Hybrid Reciprocal Velocity Obstacle paper:
    // https://gamma.cs.unc.edu/HRVO/HRVO-T-RO.pdf
    computePreferredVelocity();
    computeNeighbors();

    velocityObstacles_.clear();
    velocityObstacles_.reserve(neighbors_.size());

    // Create Velocity Obstacles for neighbors
    for (const auto &neighbor : neighbors_)
    {
        std::shared_ptr<Agent> other_agent = simulator_->getAgents()[neighbor.second];
        VelocityObstacle velocity_obstacle = other_agent->createVelocityObstacle(*this);
        velocityObstacles_.push_back(velocity_obstacle);
    }

    candidates_.clear();
    Candidate candidate;
    candidate.velocityObstacle1_ = std::numeric_limits<int>::max();
    candidate.velocityObstacle2_ = std::numeric_limits<int>::max();

    // verifies that the candidate speed is realistic and adds it to the possible
    // candidates
    auto addToCandidateListIfValid = [&](const Candidate &c) {
        if (c.velocity.lengthSquared() < max_speed_ * max_speed_)
        {
            candidates_.insert(
                std::make_pair((pref_velocity_ - c.velocity).lengthSquared(), c));
        }
    };

    // if small enough, add preferred velocity as candidate velocity or a normalized
    // version of it otherwise
    if (pref_velocity_.lengthSquared() < max_speed_ * max_speed_)
    {
        candidate.velocity = pref_velocity_;
    }
    else
    {
        candidate.velocity = pref_velocity_.normalize(max_speed_);
    }

    candidates_.insert(
        std::make_pair((pref_velocity_ - candidate.velocity).lengthSquared(), candidate));

    // this adds candidate points that are projections of the preferred velocity onto the
    // line segment of each obstacle
    for (int i = 0; i < static_cast<int>(velocityObstacles_.size()); ++i)
    {
        const Vector apex_to_pref_velocity = pref_velocity_ - velocityObstacles_[i].apex_;

        candidate.velocityObstacle1_ = i;
        candidate.velocityObstacle2_ = i;

        const float dotProduct1 =
            apex_to_pref_velocity.dot(velocityObstacles_[i].right_side);
        const float dotProduct2 =
            apex_to_pref_velocity.dot(velocityObstacles_[i].left_side);

        if (dotProduct1 > 0.0f &&
            velocityObstacles_[i].right_side.isToTheRightOf(apex_to_pref_velocity))
        {
            candidate.velocity = velocityObstacles_[i].apex_ +
                                 dotProduct1 * velocityObstacles_[i].right_side;

            addToCandidateListIfValid(candidate);
        }

        if (dotProduct2 > 0.0f &&
            velocityObstacles_[i].left_side.isToTheLeftOf(apex_to_pref_velocity))
        {
            candidate.velocity = velocityObstacles_[i].apex_ +
                                 dotProduct2 * velocityObstacles_[i].left_side;

            addToCandidateListIfValid(candidate);
        }
    }

    for (int j = 0; j < static_cast<int>(velocityObstacles_.size()); ++j)
    {
        candidate.velocityObstacle1_ = std::numeric_limits<int>::max();
        candidate.velocityObstacle2_ = j;

        float discriminant = max_speed_ * max_speed_ -
                             std::pow((velocityObstacles_[j].apex_)
                                          .determinant(velocityObstacles_[j].right_side),
                                      2.f);

        if (discriminant > 0.0f)
        {
            const float t1 =
                -(velocityObstacles_[j].apex_.dot(velocityObstacles_[j].right_side)) +
                std::sqrt(discriminant);
            const float t2 =
                -(velocityObstacles_[j].apex_.dot(velocityObstacles_[j].right_side)) -
                std::sqrt(discriminant);

            if (t1 >= 0.0f)
            {
                candidate.velocity =
                    velocityObstacles_[j].apex_ + t1 * velocityObstacles_[j].right_side;
                candidates_.insert(std::make_pair(
                    (pref_velocity_ - candidate.velocity).lengthSquared(), candidate));
            }

            if (t2 >= 0.0f)
            {
                candidate.velocity =
                    velocityObstacles_[j].apex_ + t2 * velocityObstacles_[j].right_side;
                candidates_.insert(std::make_pair(
                    (pref_velocity_ - candidate.velocity).lengthSquared(), candidate));
            }
        }

        discriminant = max_speed_ * max_speed_ -
                       std::pow((velocityObstacles_[j].apex_)
                                    .determinant(velocityObstacles_[j].left_side),
                                2.f);

        if (discriminant > 0.0f)
        {
            const float t1 =
                -(velocityObstacles_[j].apex_.dot(velocityObstacles_[j].left_side)) +
                std::sqrt(discriminant);
            const float t2 =
                -(velocityObstacles_[j].apex_.dot(velocityObstacles_[j].left_side)) -
                std::sqrt(discriminant);

            if (t1 >= 0.0f)
            {
                candidate.velocity =
                    velocityObstacles_[j].apex_ + t1 * velocityObstacles_[j].left_side;
                candidates_.insert(std::make_pair(
                    (pref_velocity_ - candidate.velocity).lengthSquared(), candidate));
            }

            if (t2 >= 0.0f)
            {
                candidate.velocity =
                    velocityObstacles_[j].apex_ + t2 * velocityObstacles_[j].left_side;
                candidates_.insert(std::make_pair(
                    (pref_velocity_ - candidate.velocity).lengthSquared(), candidate));
            }
        }
    }

    // intersection points of all velocity obstacles with each other
    for (int i = 0; i < static_cast<int>(velocityObstacles_.size()) - 1; ++i)
    {
        for (int j = i + 1; j < static_cast<int>(velocityObstacles_.size()); ++j)
        {
            candidate.velocityObstacle1_ = i;
            candidate.velocityObstacle2_ = j;

            float d = (velocityObstacles_[i].right_side)
                          .determinant(velocityObstacles_[j].right_side);

            if (d != 0.0f)
            {
                const float s =
                    (velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_)
                        .determinant(velocityObstacles_[j].right_side) /
                    d;
                const float t =
                    (velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_)
                        .determinant(velocityObstacles_[i].right_side) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate.velocity = velocityObstacles_[i].apex_ +
                                         s * velocityObstacles_[i].right_side;
                    addToCandidateListIfValid(candidate);
                }
            }

            d = (velocityObstacles_[i].left_side)
                    .determinant(velocityObstacles_[j].right_side);

            if (d != 0.0f)
            {
                const float s =
                    (velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_)
                        .determinant(velocityObstacles_[j].right_side) /
                    d;
                const float t =
                    (velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_)
                        .determinant(velocityObstacles_[i].left_side) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate.velocity =
                        velocityObstacles_[i].apex_ + s * velocityObstacles_[i].left_side;
                    addToCandidateListIfValid(candidate);
                }
            }

            d = (velocityObstacles_[i].right_side)
                    .determinant(velocityObstacles_[j].left_side);

            if (d != 0.0f)
            {
                const float s =
                    (velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_)
                        .determinant(velocityObstacles_[j].left_side) /
                    d;
                const float t =
                    (velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_)
                        .determinant(velocityObstacles_[i].right_side) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate.velocity = velocityObstacles_[i].apex_ +
                                         s * velocityObstacles_[i].right_side;
                    addToCandidateListIfValid(candidate);
                }
            }

            d = (velocityObstacles_[i].left_side)
                    .determinant(velocityObstacles_[j].left_side);

            if (d != 0.0f)
            {
                const float s =
                    (velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_)
                        .determinant(velocityObstacles_[j].left_side) /
                    d;
                const float t =
                    (velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_)
                        .determinant(velocityObstacles_[i].left_side) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate.velocity =
                        velocityObstacles_[i].apex_ + s * velocityObstacles_[i].left_side;
                    addToCandidateListIfValid(candidate);
                }
            }
        }
    }

    double min_pref_speed = pref_velocity_.length() * MIN_PREF_SPEED_MULTIPLER;

    // corresponds to the velocity obstacle that is furthest away when picking a velocity
    // we might collide with
    int optimal_furthest_away_obstacle = -1;

    new_velocity_ = Vector();
    // returns the first velocity obstacle in velocityObstacles_ that the given candidate
    // velocity intersects. returns -1 if it doesn't intersect any velocity obstacle
    auto firstIntersectingVelocityObstacle = [&](const Candidate &c) -> int {
        for (int j = 0; j < static_cast<int>(velocityObstacles_.size()); ++j)
        {
            if (j != c.velocityObstacle1_ && j != c.velocityObstacle2_ &&
                velocityObstacles_[j].containsVelocity(c.velocity))
            {
                return j;
            }
        }

        return -1;
    };

    // Choosing a candidate velocity has these goals:
    // - Pick a velocity as close as possible to the preferred velocity as long as it
    // doesn't lie in any velocity 	 	obstacle
    // - Failing the first condition, then choose the best candidate velocity as new
    // velocity that minimizes collisions 		with the closest velocity obstacles
    // - Candidate multimap is organized by distance from preferred velocity so we pick
    // the first valid velocity
    for (const auto [dist_to_pref_velocity_sq, candidate] : candidates_)
    {
        int first_intersecting_velocity_obstacle =
            firstIntersectingVelocityObstacle(candidate);
        bool is_free_of_velocity_obstacle_intersections_and_fast =
            first_intersecting_velocity_obstacle == -1 &&
            candidate.velocity.length() >= min_pref_speed;
        bool
            is_free_of_velocity_obstacle_intersections_and_slow_but_faster_than_current_selected_speed =
                first_intersecting_velocity_obstacle == -1 &&
                candidate.velocity.length() < min_pref_speed &&
                candidate.velocity.length() > new_velocity_.length();
        bool
            is_intersecting_a_velocity_obstacle_further_away_than_the_current_optimal_and_fast =
                first_intersecting_velocity_obstacle > optimal_furthest_away_obstacle &&
                candidate.velocity.length() >= min_pref_speed;
        bool
            is_intersecting_a_vo_further_away_than_optimal_and_slow_but_faster_than_current_best =
                first_intersecting_velocity_obstacle == -1 &&
                candidate.velocity.length() < min_pref_speed &&
                candidate.velocity.length() >= new_velocity_.length();

        // return as soon as we find a candidate velocity that doesn't intersect anything
        // and is fast
        if (is_free_of_velocity_obstacle_intersections_and_fast)
        {
            new_velocity_ = candidate.velocity;
            return;
        }

        // if this candidate velocity is flawed, but is better than the one we have so
        // far, store it until we find something better
        if (is_free_of_velocity_obstacle_intersections_and_slow_but_faster_than_current_selected_speed ||
            is_intersecting_a_velocity_obstacle_further_away_than_the_current_optimal_and_fast ||
            is_intersecting_a_vo_further_away_than_optimal_and_slow_but_faster_than_current_best)
        {
            optimal_furthest_away_obstacle = first_intersecting_velocity_obstacle;
            new_velocity_                  = candidate.velocity;
        }
    }
}

void HRVOAgent::computePreferredVelocity()
{
    auto path_point_opt = path.getCurrentPathPoint();

    if (prefSpeed_ <= 0.01f || max_accel_ <= 0.01f || path_point_opt == std::nullopt)
    {
        // Used to avoid edge cases with division by zero
        pref_velocity_ = Vector(0.f, 0.f);
        return;
    }

    Vector goalPosition = path_point_opt.value().getPosition();
    float speedAtGoal   = path_point_opt.value().getSpeed();

    Vector distVectorToGoal = goalPosition - position_;
    auto distToGoal         = static_cast<float>(distVectorToGoal.length());

    // d = (Vf^2 - Vi^2) / 2a
    double startLinearDecelerationDistance =
        std::abs((std::pow(speedAtGoal, 2) - std::pow(prefSpeed_, 2)) /
                 (2 * max_accel_)) *
        decel_dist_multiplier;

    if (distToGoal < startLinearDecelerationDistance)
    {
        // velocity given linear deceleration, distance away from goal, and desired final
        // speed
        // v_pref = sqrt(v_goal^2 + 2 * a * d_remainingToDestination)
        float currPrefSpeed = static_cast<float>(std::sqrt(std::pow(speedAtGoal, 2) +
                                                           2 * max_accel_ * distToGoal)) *
                              decel_pref_speed_multiplier;
        Vector ideal_pref_velocity = distVectorToGoal.normalize(currPrefSpeed);

        // Limit the preferred velocity to the kinematic limits
        const Vector dv = ideal_pref_velocity - velocity_;
        if (dv.length() <= max_accel_ * simulator_->getTimeStep())
        {
            pref_velocity_ = ideal_pref_velocity;
        }
        else
        {
            // Calculate the maximum velocity towards the preferred velocity, given the
            // acceleration constraint
            pref_velocity_ =
                velocity_ + dv.normalize(max_accel_ * simulator_->getTimeStep());
        }
    }
    else
    {
        // Accelerate to preferred speed
        // v_pref = v_now + a * t
        float currPrefSpeed =
            std::min(static_cast<double>(prefSpeed_),
                     velocity_.length() + max_accel_ * simulator_->getTimeStep());
        pref_velocity_ = distVectorToGoal.normalize(currPrefSpeed);
    }
}

void HRVOAgent::insertNeighbor(std::size_t agentNo, float &rangeSq)
{
    std::shared_ptr<Agent> other_agent = simulator_->getAgents()[agentNo];
    auto path_point_opt                = path.getCurrentPathPoint();

    if (path_point_opt == std::nullopt || this == other_agent.get())
    {
        return;
    }

    Vector other_agent_relative_pos = other_agent->getPosition() - position_;
    const float distSq              = other_agent_relative_pos.lengthSquared();

    Vector goal_pos          = path_point_opt.value().getPosition();
    Vector relative_goal_pos = goal_pos - position_;

    // Whether the other robot is with in 45 degrees of the goal, relative to us
    const float forty_five_deg_ratio = 1.f / std::sqrt(2.f);
    bool is_other_agent_in_front =
        relative_goal_pos.normalize().dot(other_agent_relative_pos.normalize()) >
        forty_five_deg_ratio;

    bool is_other_agent_moving_towards_us =
        velocity_.normalize().dot((other_agent->getVelocity()).normalize()) <
        -forty_five_deg_ratio;

    // Whether the other agent is within a 1.5-meter radius of our goal
    bool is_other_agent_near_goal =
        (other_agent->getPosition() - goal_pos).lengthSquared() < 1.5f;

    // Helper lambda function for adding other_agent to list of neighbors
    auto add_other_agent = [&]() {
        if (neighbors_.size() == maxNeighbors_)
        {
            neighbors_.erase(--neighbors_.end());
        }

        neighbors_.insert(std::make_pair(distSq, agentNo));

        if (neighbors_.size() == maxNeighbors_)
        {
            rangeSq = (--neighbors_.end())->first;
        }
    };

    if (distSq < std::pow(radius_ + other_agent->getRadius(), 2))
    {
        // In collision with other agent, so the other neighbors are not important
        neighbors_.clear();
        add_other_agent();
    }
    else if (distSq < rangeSq)
    {
        add_other_agent();
    }
    else if (is_other_agent_in_front && is_other_agent_moving_towards_us &&
             is_other_agent_near_goal)
    {
        // This is an edge case for when the other agent is outside our search range,
        // but is moving towards us from behind our destination, so it is posing a
        // possible threat of collision if we ignore it.
        add_other_agent();
    }
}

std::vector<Polygon> HRVOAgent::getVelocityObstaclesAsPolygons() const
{
    std::vector<Polygon> velocity_obstacles;
    for (const VelocityObstacle &vo : velocityObstacles_)
    {
        std::vector<Point> points;
        Vector shifted_apex  = position_ + vo.apex_;
        Vector shifted_side1 = position_ + vo.right_side;
        Vector shifted_side2 = position_ + vo.left_side;
        points.emplace_back(Point(shifted_apex.x(), shifted_apex.y()));
        points.emplace_back(Point(shifted_side1.x(), shifted_side1.y()));
        points.emplace_back(Point(shifted_side2.x(), shifted_side2.y()));
        velocity_obstacles.emplace_back(Polygon(points));
    }
    return velocity_obstacles;
}

std::vector<Circle> HRVOAgent::getCandidateVelocitiesAsCircles(
    const float circle_rad) const
{
    std::vector<Circle> candidate_circles;
    for (auto &candidate : candidates_)
    {
        Vector candidate_pos = position_ + candidate.second.velocity;
        candidate_circles.emplace_back(Circle(Point(candidate_pos), circle_rad));
    }
    return candidate_circles;
}

void HRVOAgent::setPreferredSpeed(float new_pref_speed)
{
    prefSpeed_ = new_pref_speed;
}

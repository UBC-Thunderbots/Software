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
#include "proto/message_translation/tbots_geometry.h"
#include "software/geom/vector.h"

HRVOAgent::HRVOAgent(HRVOSimulator *simulator, const Vector &position,
                     float neighbor_dist, std::size_t max_neighbors, float radius,
                     float max_radius_inflation, const Vector &velocity, float max_accel,
                     AgentPath &path, float pref_speed, float max_speed,
                     float uncertainty_offset)
    : Agent(simulator, position, radius, max_radius_inflation, velocity, velocity,
            max_speed, max_accel, path),
      max_neighbors_(max_neighbors),
      neighbor_dist_(neighbor_dist),
      pref_speed_(pref_speed),
      uncertainty_offset_(uncertainty_offset)
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
        std::min(static_cast<double>(neighbor_dist_),
                 (position_ - current_dest).length() + path.getPathRadius());

    simulator_->getKdTree()->query(this, new_neighbor_dist);
}

VelocityObstacle HRVOAgent::createVelocityObstacle(const Agent &other_agent)
{
    if ((position_ - other_agent.getPosition()).lengthSquared() >
        std::pow(radius_ + other_agent.getRadius(), 2))
    {
        // This Agent is not colliding with other agent
        const float angle =
            (position_ - other_agent.getPosition()).orientation().toRadians();

        // The opening angle of the velocity obstacle
        // opening angle = arcsin((rad_A + rad_B) / distance_BA)
        const float opening_angle =
            std::asin((radius_ + other_agent.getRadius()) /
                      (position_ - other_agent.getPosition()).length());
        // Direction of the two edges of the velocity obstacles
        Vector right_side =
            Vector::createFromAngle(Angle::fromRadians(angle - opening_angle));
        Vector left_side =
            Vector::createFromAngle(Angle::fromRadians(angle + opening_angle));

        const float d = std::sin(2.f * opening_angle);

        // This shifts one side of the velocity obstacle to share the responsibility
        // of avoiding collision with other agent. This assumes that other agent will also
        // be running HRVO
        if ((position_ - other_agent.getPosition())
                .determinant(other_agent.getPrefVelocity() - pref_velocity_) > 0.0f)
        {
            // Relative velocity is in the right half of velocity obstacle (VO)
            // Shift the VO apex to the left so the right side is smaller, making the
            // VO a Hybrid Reciprocal Velocity Obstacle (HRVO)
            const float s =
                0.5f * (other_agent.getVelocity() - velocity_).determinant(left_side) / d;

            Vector apex =
                velocity_ + s * right_side -
                (position_ - other_agent.getPosition())
                    .normalize((uncertainty_offset_ *
                                (position_ - other_agent.getPosition()).length() /
                                (radius_ + other_agent.getRadius())));
            return VelocityObstacle(apex, right_side, left_side);
        }
        else
        {
            // Relative velocity is in the left half of velocity obstacle (VO)
            // Shift the VO apex to the right so the left side is smaller, making the
            // VO a Hybrid Reciprocal Velocity Obstacle (HRVO)
            const float s =
                0.5f * (other_agent.getVelocity() - velocity_).determinant(right_side) /
                d;

            Vector apex =
                velocity_ + s * left_side -
                (position_ - other_agent.getPosition())
                    .normalize(uncertainty_offset_ *
                               (position_ - other_agent.getPosition()).length() /
                               (other_agent.getRadius() + radius_));
            return VelocityObstacle(apex, right_side, left_side);
        }
    }
    // This Agent is colliding with other agent
    // Uses Reciprocal Velocity Obstacle (RVO) with the sides being 180 degrees
    // apart from each other
    Vector apex = 0.5f * (other_agent.getVelocity() + velocity_) -
                  (position_ - other_agent.getPosition())
                      .normalize(uncertainty_offset_ +
                                 0.5f *
                                     (other_agent.getRadius() + radius_ -
                                      (position_ - other_agent.getPosition()).length()) /
                                     simulator_->getTimeStep());
    Vector right_side =
        (other_agent.getPosition() - position_).perpendicular().normalize();
    Vector left_side = -right_side;
    return VelocityObstacle(apex, right_side, left_side);
}

void HRVOAgent::computeNewVelocity()
{
    // Based on The Hybrid Reciprocal Velocity Obstacle paper:
    // https://gamma.cs.unc.edu/HRVO/HRVO-T-RO.pdf
    computePreferredVelocity();
    computeNeighbors();

    velocity_obstacles_.clear();
    velocity_obstacles_.reserve(neighbors_.size());

    // Create Velocity Obstacles for neighbors
    for (const auto &neighbor : neighbors_)
    {
        std::shared_ptr<Agent> other_agent = simulator_->getAgents()[neighbor.second];
        VelocityObstacle velocity_obstacle = other_agent->createVelocityObstacle(*this);
        velocity_obstacles_.push_back(velocity_obstacle);
    }

    candidates_.clear();
    Candidate candidate;
    candidate.velocity_obstacle_1_ = std::numeric_limits<int>::max();
    candidate.velocity_obstacle_2_ = std::numeric_limits<int>::max();

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
    for (int i = 0; i < static_cast<int>(velocity_obstacles_.size()); ++i)
    {
        const Vector apex_to_pref_velocity =
            pref_velocity_ - velocity_obstacles_[i].getApex();

        candidate.velocity_obstacle_1_ = i;
        candidate.velocity_obstacle_2_ = i;

        const float dot_product_1 =
            apex_to_pref_velocity.dot(velocity_obstacles_[i].getRightSide());
        const float dot_product_2 =
            apex_to_pref_velocity.dot(velocity_obstacles_[i].getLeftSide());

        if (dot_product_1 > 0.0f &&
            velocity_obstacles_[i].getRightSide().isClockwiseOf(apex_to_pref_velocity))
        {
            candidate.velocity = velocity_obstacles_[i].getApex() +
                                 dot_product_1 * velocity_obstacles_[i].getRightSide();

            addToCandidateListIfValid(candidate);
        }

        if (dot_product_2 > 0.0f &&
            velocity_obstacles_[i].getLeftSide().isCounterClockwiseOf(
                apex_to_pref_velocity))
        {
            candidate.velocity = velocity_obstacles_[i].getApex() +
                                 dot_product_2 * velocity_obstacles_[i].getLeftSide();

            addToCandidateListIfValid(candidate);
        }
    }

    for (int j = 0; j < static_cast<int>(velocity_obstacles_.size()); ++j)
    {
        candidate.velocity_obstacle_1_ = std::numeric_limits<int>::max();
        candidate.velocity_obstacle_2_ = j;

        float discriminant =
            max_speed_ * max_speed_ -
            std::pow((velocity_obstacles_[j].getApex())
                         .determinant(velocity_obstacles_[j].getRightSide()),
                     2.f);

        if (discriminant > 0.0f)
        {
            const float t1 = -(velocity_obstacles_[j].getApex().dot(
                                 velocity_obstacles_[j].getRightSide())) +
                             std::sqrt(discriminant);
            const float t2 = -(velocity_obstacles_[j].getApex().dot(
                                 velocity_obstacles_[j].getRightSide())) -
                             std::sqrt(discriminant);

            if (t1 >= 0.0f)
            {
                candidate.velocity = velocity_obstacles_[j].getApex() +
                                     t1 * velocity_obstacles_[j].getRightSide();
                candidates_.insert(std::make_pair(
                    (pref_velocity_ - candidate.velocity).lengthSquared(), candidate));
            }

            if (t2 >= 0.0f)
            {
                candidate.velocity = velocity_obstacles_[j].getApex() +
                                     t2 * velocity_obstacles_[j].getRightSide();
                candidates_.insert(std::make_pair(
                    (pref_velocity_ - candidate.velocity).lengthSquared(), candidate));
            }
        }

        discriminant = max_speed_ * max_speed_ -
                       std::pow((velocity_obstacles_[j].getApex())
                                    .determinant(velocity_obstacles_[j].getLeftSide()),
                                2.f);

        if (discriminant > 0.0f)
        {
            const float t1 = -(velocity_obstacles_[j].getApex().dot(
                                 velocity_obstacles_[j].getLeftSide())) +
                             std::sqrt(discriminant);
            const float t2 = -(velocity_obstacles_[j].getApex().dot(
                                 velocity_obstacles_[j].getLeftSide())) -
                             std::sqrt(discriminant);

            if (t1 >= 0.0f)
            {
                candidate.velocity = velocity_obstacles_[j].getApex() +
                                     t1 * velocity_obstacles_[j].getLeftSide();
                candidates_.insert(std::make_pair(
                    (pref_velocity_ - candidate.velocity).lengthSquared(), candidate));
            }

            if (t2 >= 0.0f)
            {
                candidate.velocity = velocity_obstacles_[j].getApex() +
                                     t2 * velocity_obstacles_[j].getLeftSide();
                candidates_.insert(std::make_pair(
                    (pref_velocity_ - candidate.velocity).lengthSquared(), candidate));
            }
        }
    }

    // intersection points of all velocity obstacles with each other
    for (int i = 0; i < static_cast<int>(velocity_obstacles_.size()) - 1; ++i)
    {
        for (int j = i + 1; j < static_cast<int>(velocity_obstacles_.size()); ++j)
        {
            candidate.velocity_obstacle_1_ = i;
            candidate.velocity_obstacle_2_ = j;

            float d = (velocity_obstacles_[i].getRightSide())
                          .determinant(velocity_obstacles_[j].getRightSide());

            if (d != 0.0f)
            {
                const float s =
                    (velocity_obstacles_[j].getApex() - velocity_obstacles_[i].getApex())
                        .determinant(velocity_obstacles_[j].getRightSide()) /
                    d;
                const float t =
                    (velocity_obstacles_[j].getApex() - velocity_obstacles_[i].getApex())
                        .determinant(velocity_obstacles_[i].getRightSide()) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate.velocity = velocity_obstacles_[i].getApex() +
                                         s * velocity_obstacles_[i].getRightSide();
                    addToCandidateListIfValid(candidate);
                }
            }

            d = (velocity_obstacles_[i].getLeftSide())
                    .determinant(velocity_obstacles_[j].getRightSide());

            if (d != 0.0f)
            {
                const float s =
                    (velocity_obstacles_[j].getApex() - velocity_obstacles_[i].getApex())
                        .determinant(velocity_obstacles_[j].getRightSide()) /
                    d;
                const float t =
                    (velocity_obstacles_[j].getApex() - velocity_obstacles_[i].getApex())
                        .determinant(velocity_obstacles_[i].getLeftSide()) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate.velocity = velocity_obstacles_[i].getApex() +
                                         s * velocity_obstacles_[i].getLeftSide();
                    addToCandidateListIfValid(candidate);
                }
            }

            d = (velocity_obstacles_[i].getRightSide())
                    .determinant(velocity_obstacles_[j].getLeftSide());

            if (d != 0.0f)
            {
                const float s =
                    (velocity_obstacles_[j].getApex() - velocity_obstacles_[i].getApex())
                        .determinant(velocity_obstacles_[j].getLeftSide()) /
                    d;
                const float t =
                    (velocity_obstacles_[j].getApex() - velocity_obstacles_[i].getApex())
                        .determinant(velocity_obstacles_[i].getRightSide()) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate.velocity = velocity_obstacles_[i].getApex() +
                                         s * velocity_obstacles_[i].getRightSide();
                    addToCandidateListIfValid(candidate);
                }
            }

            d = (velocity_obstacles_[i].getLeftSide())
                    .determinant(velocity_obstacles_[j].getLeftSide());

            if (d != 0.0f)
            {
                const float s =
                    (velocity_obstacles_[j].getApex() - velocity_obstacles_[i].getApex())
                        .determinant(velocity_obstacles_[j].getLeftSide()) /
                    d;
                const float t =
                    (velocity_obstacles_[j].getApex() - velocity_obstacles_[i].getApex())
                        .determinant(velocity_obstacles_[i].getLeftSide()) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate.velocity = velocity_obstacles_[i].getApex() +
                                         s * velocity_obstacles_[i].getLeftSide();
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

    // Choosing a candidate velocity has these goals:
    // - Pick a velocity as close as possible to the preferred velocity as long as it
    // doesn't lie in any velocity obstacle
    // - Failing the first condition, then choose the best candidate velocity as the new
    // velocity that minimizes collisions with the closest velocity obstacles
    // - Candidate multimap is organized by distance from preferred velocity so we pick
    // the first valid velocity
    for (const auto [dist_to_pref_velocity_sq, candidate] : candidates_)
    {
        std::optional<int> first_intersecting_velocity_obstacle =
            findIntersectingVelocityObstacle(candidate);

        // return as soon as we find a candidate velocity that doesn't intersect anything
        // and is fast
        if (isIdealCandidate(candidate))
        {
            new_velocity_ = candidate.velocity;
            return;
        }

        // if this candidate velocity is flawed, but is better than the one we have so
        // far, store it until we find something better
        // these velocities have one of the following characteristics:
        // - the candidate velocity doesn't intersect any velocity obstacle but is slow
        // - the candidate velocity intersects a velocity obstacle further away than the
        // current best obstacle and is fast
        // - the candidate velocity intersects a velocity obstacle further away than the
        // current best obstacle but is	slow yet still faster than the current best
        // velocity
        if ((!first_intersecting_velocity_obstacle.has_value() &&
             isCandidateSlow(candidate) &&
             isCandidateFasterThanCurrentSpeed(candidate)) ||
            (first_intersecting_velocity_obstacle.has_value() &&
             first_intersecting_velocity_obstacle > optimal_furthest_away_obstacle &&
             isCandidateFast(candidate)) ||
            (first_intersecting_velocity_obstacle.has_value() &&
             first_intersecting_velocity_obstacle > optimal_furthest_away_obstacle &&
             isCandidateSlow(candidate) && isCandidateFasterThanCurrentSpeed(candidate)))
        {
            if (first_intersecting_velocity_obstacle.has_value())
            {
                optimal_furthest_away_obstacle =
                    first_intersecting_velocity_obstacle.value();
            }
            new_velocity_ = candidate.velocity;
        }
    }
}

bool HRVOAgent::isIdealCandidate(const Candidate &candidate) const
{
    return !findIntersectingVelocityObstacle(candidate).has_value() &&
           isCandidateFast(candidate);
}

bool HRVOAgent::isCandidateSlow(const Candidate &candidate) const
{
    double min_pref_speed = std::abs(pref_velocity_.length()) * MIN_PREF_SPEED_MULTIPLER;

    return std::abs(candidate.velocity.length()) < min_pref_speed;
}

bool HRVOAgent::isCandidateFast(const Candidate &candidate) const
{
    return !isCandidateSlow(candidate);
}

bool HRVOAgent::isCandidateFasterThanCurrentSpeed(const Candidate &candidate) const
{
    return std::abs(candidate.velocity.length()) > std::abs(new_velocity_.length());
}

std::optional<int> HRVOAgent::findIntersectingVelocityObstacle(
    const Candidate &candidate) const
{
    for (int j = 0; j < static_cast<int>(velocity_obstacles_.size()); ++j)
    {
        if (j != candidate.velocity_obstacle_1_ && j != candidate.velocity_obstacle_2_ &&
            velocity_obstacles_[j].containsVelocity(candidate.velocity))
        {
            return std::make_optional<int>(j);
        }
    }

    return std::nullopt;
}

void HRVOAgent::computePreferredVelocity()
{
    auto path_point_opt = path.getCurrentPathPoint();

    if (pref_speed_ <= 0.01f || max_accel_ <= 0.01f || path_point_opt == std::nullopt)
    {
        // Used to avoid edge cases with division by zero
        pref_velocity_ = Vector(0.f, 0.f);
        return;
    }

    Vector goal_position = path_point_opt.value().getPosition();
    float speed_at_goal  = path_point_opt.value().getSpeed();

    Vector dist_vector_to_goal = goal_position - position_;
    auto dist_to_goal          = static_cast<float>(dist_vector_to_goal.length());

    // d = (Vf^2 - Vi^2) / 2a
    double start_linear_deceleration_distance =
        std::abs((std::pow(speed_at_goal, 2) - std::pow(pref_speed_, 2)) /
                 (2 * max_accel_)) *
        decel_dist_multiplier;

    if (dist_to_goal < start_linear_deceleration_distance)
    {
        // velocity given linear deceleration, distance away from goal, and desired final
        // speed
        // v_pref = sqrt(v_goal^2 + 2 * a * d_remainingToDestination)
        float curr_pref_speed =
            static_cast<float>(
                std::sqrt(std::pow(speed_at_goal, 2) + 2 * max_accel_ * dist_to_goal)) *
            decel_pref_speed_multiplier;
        Vector ideal_pref_velocity = dist_vector_to_goal.normalize(curr_pref_speed);

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
        float curr_pref_speed =
            std::min(static_cast<double>(pref_speed_),
                     velocity_.length() + max_accel_ * simulator_->getTimeStep());
        pref_velocity_ = dist_vector_to_goal.normalize(curr_pref_speed);
    }
}

void HRVOAgent::insertNeighbor(std::size_t agent_no, float &range_sq)
{
    std::shared_ptr<Agent> other_agent = simulator_->getAgents()[agent_no];
    auto path_point_opt                = path.getCurrentPathPoint();

    if (path_point_opt == std::nullopt || this == other_agent.get())
    {
        return;
    }

    Vector other_agent_relative_pos = other_agent->getPosition() - position_;
    const float dist_sq             = other_agent_relative_pos.lengthSquared();

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
        if (neighbors_.size() == max_neighbors_)
        {
            neighbors_.erase(--neighbors_.end());
        }

        neighbors_.insert(std::make_pair(dist_sq, agent_no));

        if (neighbors_.size() == max_neighbors_)
        {
            range_sq = (--neighbors_.end())->first;
        }
    };

    if (dist_sq < std::pow(radius_ + other_agent->getRadius(), 2))
    {
        // In collision with other agent, so the other neighbors are not important
        neighbors_.clear();
        add_other_agent();
    }
    else if (dist_sq < range_sq)
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

std::vector<TbotsProto::VelocityObstacle> HRVOAgent::getVelocityObstaclesAsProto() const
{
    std::vector<TbotsProto::VelocityObstacle> velocity_obstacles;
    for (const VelocityObstacle &vo : velocity_obstacles_)
    {
        velocity_obstacles.emplace_back(*createVelocityObstacleProto(vo, getPosition()));
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
    pref_speed_ = new_pref_speed;
}

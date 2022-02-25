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

#include "goal.h"
#include "kd_tree.h"
#include "software/geom/vector.h"


HRVOAgent::HRVOAgent(Simulator *simulator, const Vector &position, std::size_t goalIndex,
                     float neighborDist, std::size_t maxNeighbors, float radius,
                     const Vector &velocity, float maxAccel, float goalRadius,
                     float prefSpeed, float maxSpeed, float uncertaintyOffset)
    : Agent(simulator, position, radius, velocity, velocity, maxSpeed, maxAccel,
            goalIndex, goalRadius),
      maxNeighbors_(maxNeighbors),
      neighborDist_(neighborDist),
      prefSpeed_(prefSpeed),
      uncertaintyOffset_(uncertaintyOffset)
{
}

void HRVOAgent::computeNeighbors()
{
    neighbors_.clear();

    std::unique_ptr<Goal> &current_goal = simulator_->goals_[goal_index_];
    float new_neighbor_dist             = std::min(
        static_cast<double>(neighborDist_),
        (position_ - current_goal->getCurrentGoalPosition()).length() + goal_radius_);

    simulator_->kdTree_->query(this, new_neighbor_dist);
}

Agent::VelocityObstacle HRVOAgent::createVelocityObstacle(const Agent &other_agent)
{
    VelocityObstacle velocityObstacle;
    if ((position_ - other_agent.getPosition()).lengthSquared() >
        std::pow(radius_ + other_agent.getRadius(), 2))
    {
        // This Agent is not colliding with other agent
        const float angle = atan2((position_ - other_agent.getPosition()).y(),
                                  (position_ - other_agent.getPosition()).x());

        // The opening angle of the velocity obstacle
        // opening angle = arcsin((rad_A + rad_B) / distance_BA)
        const float openingAngle =
            std::asin((radius_ + other_agent.getRadius()) /
                      (position_ - other_agent.getPosition()).length());

        // Direction of the two edges of the velocity obstacle
        velocityObstacle.side1_ =
            Vector(std::cos(angle - openingAngle), std::sin(angle - openingAngle));
        velocityObstacle.side2_ =
            Vector(std::cos(angle + openingAngle), std::sin(angle + openingAngle));

        const float d = std::sin(2.f * openingAngle);

        // This shifts one side of the velocity obstacle to share the responsibility
        // of avoiding collision with other agent. This assumes that other agent will also
        // be running HRVO
        if ((position_ - other_agent.getPosition())
                .det(other_agent.getPrefVelocity() - pref_velocity_) > 0.0f)
        {
            // Relative velocity is in the right half of velocity obstacle (VO)
            // Shift the VO apex to the left so the right side is smaller, making the
            // VO a Hybrid Reciprocal Velocity Obstacle (HRVO)
            const float s =
                0.5f *
                (other_agent.getVelocity() - velocity_).det(velocityObstacle.side2_) / d;

            velocityObstacle.apex_ =
                velocity_ + s * velocityObstacle.side1_ -
                (uncertaintyOffset_ * (position_ - other_agent.getPosition()).length() /
                 (radius_ + other_agent.getRadius())) *
                    (position_ - other_agent.getPosition()).normalize();
        }
        else
        {
            // Relative velocity is in the left half of velocity obstacle (VO)
            // Shift the VO apex to the right so the left side is smaller, making the
            // VO a Hybrid Reciprocal Velocity Obstacle (HRVO)
            const float s =
                0.5f *
                (other_agent.getVelocity() - velocity_).det(velocityObstacle.side1_) / d;

            velocityObstacle.apex_ =
                velocity_ + s * velocityObstacle.side2_ -
                (uncertaintyOffset_ * (position_ - other_agent.getPosition()).length() /
                 (other_agent.getRadius() + radius_)) *
                    (position_ - other_agent.getPosition()).normalize();
        }
    }
    else
    {
        // This Agent is colliding with other agent
        // Uses Reciprocal Velocity Obstacle (RVO) with the sides being 180 degrees
        // apart from each other
        velocityObstacle.apex_ =
            0.5f * (other_agent.getVelocity() + velocity_) -
            (uncertaintyOffset_ + 0.5f *
                                      (other_agent.getRadius() + radius_ -
                                       (position_ - other_agent.getPosition()).length()) /
                                      simulator_->timeStep_) *
                (position_ - other_agent.getPosition()).normalize();
        velocityObstacle.side1_ = (other_agent.getPosition(), position_).perpendicular();
        velocityObstacle.side2_ = -velocityObstacle.side1_;
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
        const std::unique_ptr<Agent> &other_agent = simulator_->agents_[neighbor.second];
        VelocityObstacle velocity_obstacle = other_agent->createVelocityObstacle(*this);
        velocityObstacles_.push_back(velocity_obstacle);
    }

    // Calculate what velocities (candidates) are not inside any velocity obstacle
    // This is likely implementing the ClearPath efficient geometric algorithm as stated
    // in the HRVO paper to find the closest possible velocity to our preferred velocity.
    candidates_.clear();

    Candidate candidate;

    candidate.velocityObstacle1_ = std::numeric_limits<int>::max();
    candidate.velocityObstacle2_ = std::numeric_limits<int>::max();

    if (pref_velocity_.lengthSquared() < max_speed_ * max_speed_)
    {
        candidate.position_ = pref_velocity_;
    }
    else
    {
        candidate.position_ = max_speed_ * pref_velocity_.normalize();
    }

    candidates_.insert(std::make_pair(
        (pref_velocity_ - candidate.position_).lengthSquared(), candidate));

    for (int i = 0; i < static_cast<int>(velocityObstacles_.size()); ++i)
    {
        candidate.velocityObstacle1_ = i;
        candidate.velocityObstacle2_ = i;

        const float dotProduct1 = (pref_velocity_ - velocityObstacles_[i].apex_)
                                      .dot(velocityObstacles_[i].side1_);
        const float dotProduct2 = (pref_velocity_ - velocityObstacles_[i].apex_)
                                      .dot(velocityObstacles_[i].side2_);

        if (dotProduct1 > 0.0f &&
            (velocityObstacles_[i].side1_)
                    .det(pref_velocity_ - velocityObstacles_[i].apex_) > 0.0f)
        {
            candidate.position_ =
                velocityObstacles_[i].apex_ + dotProduct1 * velocityObstacles_[i].side1_;

            if (candidate.position_.lengthSquared() < max_speed_ * max_speed_)
            {
                candidates_.insert(std::make_pair(
                    (pref_velocity_ - candidate.position_).lengthSquared(), candidate));
            }
        }

        if (dotProduct2 > 0.0f &&
            (velocityObstacles_[i].side2_)
                    .det(pref_velocity_ - velocityObstacles_[i].apex_) < 0.0f)
        {
            candidate.position_ =
                velocityObstacles_[i].apex_ + dotProduct2 * velocityObstacles_[i].side2_;

            if (candidate.position_.lengthSquared() < max_speed_ * max_speed_)
            {
                candidates_.insert(std::make_pair(
                    (pref_velocity_ - candidate.position_).lengthSquared(), candidate));
            }
        }
    }

    for (int j = 0; j < static_cast<int>(velocityObstacles_.size()); ++j)
    {
        candidate.velocityObstacle1_ = std::numeric_limits<int>::max();
        candidate.velocityObstacle2_ = j;

        float discriminant =
            max_speed_ * max_speed_ -
            std::pow((velocityObstacles_[j].apex_).det(velocityObstacles_[j].side1_),
                     2.f);

        if (discriminant > 0.0f)
        {
            const float t1 =
                -(velocityObstacles_[j].apex_.dot(velocityObstacles_[j].side1_)) +
                std::sqrt(discriminant);
            const float t2 =
                -(velocityObstacles_[j].apex_.dot(velocityObstacles_[j].side1_)) -
                std::sqrt(discriminant);

            if (t1 >= 0.0f)
            {
                candidate.position_ =
                    velocityObstacles_[j].apex_ + t1 * velocityObstacles_[j].side1_;
                candidates_.insert(std::make_pair(
                    (pref_velocity_ - candidate.position_).lengthSquared(), candidate));
            }

            if (t2 >= 0.0f)
            {
                candidate.position_ =
                    velocityObstacles_[j].apex_ + t2 * velocityObstacles_[j].side1_;
                candidates_.insert(std::make_pair(
                    (pref_velocity_ - candidate.position_).lengthSquared(), candidate));
            }
        }

        discriminant =
            max_speed_ * max_speed_ -
            std::pow((velocityObstacles_[j].apex_).det(velocityObstacles_[j].side2_),
                     2.f);

        if (discriminant > 0.0f)
        {
            const float t1 =
                -(velocityObstacles_[j].apex_.dot(velocityObstacles_[j].side2_)) +
                std::sqrt(discriminant);
            const float t2 =
                -(velocityObstacles_[j].apex_.dot(velocityObstacles_[j].side2_)) -
                std::sqrt(discriminant);

            if (t1 >= 0.0f)
            {
                candidate.position_ =
                    velocityObstacles_[j].apex_ + t1 * velocityObstacles_[j].side2_;
                candidates_.insert(std::make_pair(
                    (pref_velocity_ - candidate.position_).lengthSquared(), candidate));
            }

            if (t2 >= 0.0f)
            {
                candidate.position_ =
                    velocityObstacles_[j].apex_ + t2 * velocityObstacles_[j].side2_;
                candidates_.insert(std::make_pair(
                    (pref_velocity_ - candidate.position_).lengthSquared(), candidate));
            }
        }
    }

    for (int i = 0; i < static_cast<int>(velocityObstacles_.size()) - 1; ++i)
    {
        for (int j = i + 1; j < static_cast<int>(velocityObstacles_.size()); ++j)
        {
            candidate.velocityObstacle1_ = i;
            candidate.velocityObstacle2_ = j;

            float d = (velocityObstacles_[i].side1_).det(velocityObstacles_[j].side1_);

            if (d != 0.0f)
            {
                const float s =
                    (velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_)
                        .det(velocityObstacles_[j].side1_) /
                    d;
                const float t =
                    (velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_)
                        .det(velocityObstacles_[i].side1_) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate.position_ =
                        velocityObstacles_[i].apex_ + s * velocityObstacles_[i].side1_;

                    if (candidate.position_.lengthSquared() < max_speed_ * max_speed_)
                    {
                        candidates_.insert(std::make_pair(
                            (pref_velocity_ - candidate.position_).lengthSquared(),
                            candidate));
                    }
                }
            }

            d = (velocityObstacles_[i].side2_).det(velocityObstacles_[j].side1_);

            if (d != 0.0f)
            {
                const float s =
                    (velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_)
                        .det(velocityObstacles_[j].side1_) /
                    d;
                const float t =
                    (velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_)
                        .det(velocityObstacles_[i].side2_) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate.position_ =
                        velocityObstacles_[i].apex_ + s * velocityObstacles_[i].side2_;

                    if (candidate.position_.lengthSquared() < max_speed_ * max_speed_)
                    {
                        candidates_.insert(std::make_pair(
                            (pref_velocity_ - candidate.position_).lengthSquared(),
                            candidate));
                    }
                }
            }

            d = (velocityObstacles_[i].side1_).det(velocityObstacles_[j].side2_);

            if (d != 0.0f)
            {
                const float s =
                    (velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_)
                        .det(velocityObstacles_[j].side2_) /
                    d;
                const float t =
                    (velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_)
                        .det(velocityObstacles_[i].side1_) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate.position_ =
                        velocityObstacles_[i].apex_ + s * velocityObstacles_[i].side1_;

                    if (candidate.position_.lengthSquared() < max_speed_ * max_speed_)
                    {
                        candidates_.insert(std::make_pair(
                            (pref_velocity_ - candidate.position_).lengthSquared(),
                            candidate));
                    }
                }
            }

            d = (velocityObstacles_[i].side2_).det(velocityObstacles_[j].side2_);

            if (d != 0.0f)
            {
                const float s =
                    (velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_)
                        .det(velocityObstacles_[j].side2_) /
                    d;
                const float t =
                    (velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_)
                        .det(velocityObstacles_[i].side2_) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate.position_ =
                        velocityObstacles_[i].apex_ + s * velocityObstacles_[i].side2_;

                    if (candidate.position_.lengthSquared() < max_speed_ * max_speed_)
                    {
                        candidates_.insert(std::make_pair(
                            (pref_velocity_ - candidate.position_).lengthSquared(),
                            candidate));
                    }
                }
            }
        }
    }

    int optimal = -1;

    for (std::pair<float, Candidate> candidate_pair : candidates_)
    {
        candidate  = candidate_pair.second;
        bool valid = true;

        for (int j = 0; j < static_cast<int>(velocityObstacles_.size()); ++j)
        {
            if (j != candidate.velocityObstacle1_ && j != candidate.velocityObstacle2_ &&
                (velocityObstacles_[j].side2_)
                        .det(candidate.position_ - velocityObstacles_[j].apex_) < 0.0f &&
                (velocityObstacles_[j].side1_)
                        .det(candidate.position_ - velocityObstacles_[j].apex_) > 0.0f)
            {
                valid = false;

                if (j > optimal)
                {
                    optimal       = j;
                    new_velocity_ = candidate.position_;
                }

                break;
            }
        }

        if (valid)
        {
            new_velocity_ = candidate.position_;
            break;
        }
    }
}

void HRVOAgent::computePreferredVelocity()
{
    if (prefSpeed_ <= 0.01f || max_accel_ <= 0.01f)
    {
        // Used to avoid edge cases with division by zero
        pref_velocity_ = Vector(0.f, 0.f);
        return;
    }

    std::unique_ptr<Goal> &nextGoal = simulator_->goals_[goal_index_];
    Vector goalPosition             = nextGoal->getCurrentGoalPosition();
    float speedAtGoal               = nextGoal->getDesiredSpeedAtCurrentGoal();
    Vector distVectorToGoal         = goalPosition - position_;
    auto distToGoal                 = static_cast<float>(
        std::sqrt(std::pow(distVectorToGoal.x(), 2) + std::pow(distVectorToGoal.y(), 2)));
    // d = (Vf^2 - Vi^2) / 2a
    double startLinearDecelerationDistance =
        std::abs((std::pow(speedAtGoal, 2) - std::pow(prefSpeed_, 2)) / (2 * max_accel_));

    if (distToGoal < startLinearDecelerationDistance)
    {
        // velocity given linear deceleration, distance away from goal, and desired final
        // speed
        // v_pref = sqrt(v_goal^2 + 2 * a * d_remainingToDestination)
        float currPrefSpeed = static_cast<float>(
            std::sqrt(std::pow(speedAtGoal, 2) + 2 * max_accel_ * distToGoal));
        Vector ideal_pref_velocity = distVectorToGoal.normalize() * currPrefSpeed;

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
                velocity_ + (max_accel_ * simulator_->getTimeStep()) * (dv / dv.length());
        }
    }
    else
    {
        // Accelerate to preferred speed
        // v_pref = v_now + a * t
        float currPrefSpeed =
            std::min(static_cast<double>(prefSpeed_),
                     velocity_.length() + max_accel_ * simulator_->getTimeStep());
        pref_velocity_ = distVectorToGoal.normalize() * currPrefSpeed;
    }
}

void HRVOAgent::insertNeighbor(std::size_t agentNo, float &rangeSq)
{
    const std::unique_ptr<Agent> &other_agent = simulator_->agents_[agentNo];

    if (this != other_agent.get())
    {
        Vector other_agent_relative_pos = other_agent->getPosition() - position_;
        const float distSq              = other_agent_relative_pos.lengthSquared();

        Vector goal_pos = simulator_->goals_[goal_index_]->getCurrentGoalPosition();
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

        // Helper lambda function
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

        if (distSq < std::pow(radius_ + other_agent->getRadius(), 2) && distSq < rangeSq)
        {
            // In collision with other agent, so the other neighbors are not important
            neighbors_.clear();
            add_other_agent();
        }
        else if (distSq < rangeSq)
        {
            add_other_agent();
        }
        else if (distSq > rangeSq && is_other_agent_in_front &&
                 is_other_agent_moving_towards_us && is_other_agent_near_goal)
        {
            // This is an edge case for when the other agent is outside our search range,
            // but is moving towards us from behind our destination, so it is posing a
            // possible threat of collision if we ignore it.
            add_other_agent();
        }
    }
}

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


HRVOAgent::HRVOAgent(Simulator *simulator, const Vector2 &position, std::size_t goalNo,
                     float neighborDist, std::size_t maxNeighbors, float radius,
                     const Vector2 &velocity, float maxAccel, float goalRadius,
                     float prefSpeed, float maxSpeed, float uncertaintyOffset)
    : Agent(simulator, position, radius, velocity, velocity, maxSpeed, maxAccel, goalNo,
            goalRadius),
      maxNeighbors_(maxNeighbors),
      neighborDist_(neighborDist),
      prefSpeed_(prefSpeed),
      uncertaintyOffset_(uncertaintyOffset)
{
}

void HRVOAgent::computeNeighbors()
{
    neighbors_.clear();
    simulator_->kdTree_->query(this, neighborDist_ * neighborDist_);
}

void HRVOAgent::computeNewVelocity()
{
    // Based on The Hybrid Reciprocal Velocity Obstacle paper:
    // https://gamma.cs.unc.edu/HRVO/HRVO-T-RO.pdf
    computePreferredVelocity();
    computeNeighbors();

    velocityObstacles_.clear();
    velocityObstacles_.reserve(neighbors_.size());

    VelocityObstacle velocityObstacle;

    // Create Hybrid Reciprocal Velocity Obstacles for neighbors
    for (const auto &neighbor : neighbors_)
    {
        const std::unique_ptr<Agent> &other = simulator_->agents_[neighbor.second];

        if (absSq(other->position_ - position_) > std::pow(other->radius_ + radius_, 2))
        {
            // This Agent is not colliding with neighbor
            const float angle = atan(other->position_ - position_);

            // opening angle = arcsin((rad_A + rad_B) / distance)
            const float openingAngle =
                std::asin((other->radius_ + radius_) / abs(other->position_ - position_));

            // Direction of the two edges of the velocity obstacle
            velocityObstacle.side1_ =
                Vector2(std::cos(angle - openingAngle), std::sin(angle - openingAngle));
            velocityObstacle.side2_ =
                Vector2(std::cos(angle + openingAngle), std::sin(angle + openingAngle));

            // Diameter = 2 * sin(openingAngle) * cos(openingAngle) = sin(2 * openingAngle) = 2 * (rad_A + rad_B) ?
            const float d = 2.0f * std::sin(openingAngle) * std::cos(openingAngle);

            // This shifts one side of the velocity obstacle to share the responsibility of avoiding collision with
            // neighbor. This assumes that neighbor will also be running HRVO
            if (det(other->position_ - position_, prefVelocity_ - other->prefVelocity_) >
                0.0f)
            {
                // Relative velocity is in the right half of velocity obstacle (VO)
                // Shift the VO apex to the left so the right side is smaller, making the VO a Hybrid Reciprocal Velocity Obstacle (HRVO)
                const float s =
                    0.5f * det(velocity_ - other->velocity_, velocityObstacle.side2_) / d;

                velocityObstacle.apex_ =
                    other->velocity_ + s * velocityObstacle.side1_ -                    // Apex +
                    (uncertaintyOffset_ * abs(other->position_ - position_) /    // Uncertainty
                     (other->radius_ + radius_)) *
                        normalize(other->position_ - position_);
            }
            else
            {
                // Relative velocity is in the left half of velocity obstacle (VO)
                // Shift the VO apex to the right so the left side is smaller, making the VO a Hybrid Reciprocal Velocity Obstacle (HRVO)
                const float s =
                    0.5f * det(velocity_ - other->velocity_, velocityObstacle.side1_) / d;

                velocityObstacle.apex_ =
                    other->velocity_ + s * velocityObstacle.side2_ -
                    (uncertaintyOffset_ * abs(other->position_ - position_) /
                     (other->radius_ + radius_)) *
                        normalize(other->position_ - position_);
            }

            velocityObstacles_.push_back(velocityObstacle);
        }
        else
        {
            // This Agent is colliding with neighbor
            // Uses Reciprocal Velocity Obstacle (RVO) with the sides being 180 degrees apart from each other
            velocityObstacle.apex_ =
                0.5f * (other->velocity_ + velocity_) -
                (uncertaintyOffset_ +
                 0.5f * (other->radius_ + radius_ - abs(other->position_ - position_)) /
                     simulator_->timeStep_) *
                    normalize(other->position_ - position_);
            velocityObstacle.side1_ = normal(position_, other->position_);
            velocityObstacle.side2_ = -velocityObstacle.side1_;
            velocityObstacles_.push_back(velocityObstacle);
        }
    }

    // Calculate what velocities (candidates) are not inside any velocity obstacle
    // This is likely implementing the ClearPath efficient geometric algorithm as stated in the HRVO paper to find the
    // closest possible velocity to our preferred velocity.
    candidates_.clear();

    Candidate candidate;

    candidate.velocityObstacle1_ = std::numeric_limits<int>::max();
    candidate.velocityObstacle2_ = std::numeric_limits<int>::max();

    if (absSq(prefVelocity_) < maxSpeed_ * maxSpeed_)
    {
        candidate.position_ = prefVelocity_;
    }
    else
    {
        candidate.position_ = maxSpeed_ * normalize(prefVelocity_);
    }

    candidates_.insert(
        std::make_pair(absSq(prefVelocity_ - candidate.position_), candidate));

    for (int i = 0; i < static_cast<int>(velocityObstacles_.size()); ++i)
    {
        candidate.velocityObstacle1_ = i;
        candidate.velocityObstacle2_ = i;

        const float dotProduct1 =
            (prefVelocity_ - velocityObstacles_[i].apex_) * velocityObstacles_[i].side1_;
        const float dotProduct2 =
            (prefVelocity_ - velocityObstacles_[i].apex_) * velocityObstacles_[i].side2_;

        if (dotProduct1 > 0.0f && det(velocityObstacles_[i].side1_,
                                      prefVelocity_ - velocityObstacles_[i].apex_) > 0.0f)
        {
            candidate.position_ =
                velocityObstacles_[i].apex_ + dotProduct1 * velocityObstacles_[i].side1_;

            if (absSq(candidate.position_) < maxSpeed_ * maxSpeed_)
            {
                candidates_.insert(std::make_pair(
                    absSq(prefVelocity_ - candidate.position_), candidate));
            }
        }

        if (dotProduct2 > 0.0f && det(velocityObstacles_[i].side2_,
                                      prefVelocity_ - velocityObstacles_[i].apex_) < 0.0f)
        {
            candidate.position_ =
                velocityObstacles_[i].apex_ + dotProduct2 * velocityObstacles_[i].side2_;

            if (absSq(candidate.position_) < maxSpeed_ * maxSpeed_)
            {
                candidates_.insert(std::make_pair(
                    absSq(prefVelocity_ - candidate.position_), candidate));
            }
        }
    }

    for (int j = 0; j < static_cast<int>(velocityObstacles_.size()); ++j)
    {
        candidate.velocityObstacle1_ = std::numeric_limits<int>::max();
        candidate.velocityObstacle2_ = j;

        float discriminant =
            maxSpeed_ * maxSpeed_ -
            std::pow(det(velocityObstacles_[j].apex_, velocityObstacles_[j].side1_), 2.f);

        if (discriminant > 0.0f)
        {
            const float t1 =
                -(velocityObstacles_[j].apex_ * velocityObstacles_[j].side1_) +
                std::sqrt(discriminant);
            const float t2 =
                -(velocityObstacles_[j].apex_ * velocityObstacles_[j].side1_) -
                std::sqrt(discriminant);

            if (t1 >= 0.0f)
            {
                candidate.position_ =
                    velocityObstacles_[j].apex_ + t1 * velocityObstacles_[j].side1_;
                candidates_.insert(std::make_pair(
                    absSq(prefVelocity_ - candidate.position_), candidate));
            }

            if (t2 >= 0.0f)
            {
                candidate.position_ =
                    velocityObstacles_[j].apex_ + t2 * velocityObstacles_[j].side1_;
                candidates_.insert(std::make_pair(
                    absSq(prefVelocity_ - candidate.position_), candidate));
            }
        }

        discriminant =
            maxSpeed_ * maxSpeed_ -
            std::pow(det(velocityObstacles_[j].apex_, velocityObstacles_[j].side2_), 2.f);

        if (discriminant > 0.0f)
        {
            const float t1 =
                -(velocityObstacles_[j].apex_ * velocityObstacles_[j].side2_) +
                std::sqrt(discriminant);
            const float t2 =
                -(velocityObstacles_[j].apex_ * velocityObstacles_[j].side2_) -
                std::sqrt(discriminant);

            if (t1 >= 0.0f)
            {
                candidate.position_ =
                    velocityObstacles_[j].apex_ + t1 * velocityObstacles_[j].side2_;
                candidates_.insert(std::make_pair(
                    absSq(prefVelocity_ - candidate.position_), candidate));
            }

            if (t2 >= 0.0f)
            {
                candidate.position_ =
                    velocityObstacles_[j].apex_ + t2 * velocityObstacles_[j].side2_;
                candidates_.insert(std::make_pair(
                    absSq(prefVelocity_ - candidate.position_), candidate));
            }
        }
    }

    for (int i = 0; i < static_cast<int>(velocityObstacles_.size()) - 1; ++i)
    {
        for (int j = i + 1; j < static_cast<int>(velocityObstacles_.size()); ++j)
        {
            candidate.velocityObstacle1_ = i;
            candidate.velocityObstacle2_ = j;

            float d = det(velocityObstacles_[i].side1_, velocityObstacles_[j].side1_);

            if (d != 0.0f)
            {
                const float s =
                    det(velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_,
                        velocityObstacles_[j].side1_) /
                    d;
                const float t =
                    det(velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_,
                        velocityObstacles_[i].side1_) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate.position_ =
                        velocityObstacles_[i].apex_ + s * velocityObstacles_[i].side1_;

                    if (absSq(candidate.position_) < maxSpeed_ * maxSpeed_)
                    {
                        candidates_.insert(std::make_pair(
                            absSq(prefVelocity_ - candidate.position_), candidate));
                    }
                }
            }

            d = det(velocityObstacles_[i].side2_, velocityObstacles_[j].side1_);

            if (d != 0.0f)
            {
                const float s =
                    det(velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_,
                        velocityObstacles_[j].side1_) /
                    d;
                const float t =
                    det(velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_,
                        velocityObstacles_[i].side2_) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate.position_ =
                        velocityObstacles_[i].apex_ + s * velocityObstacles_[i].side2_;

                    if (absSq(candidate.position_) < maxSpeed_ * maxSpeed_)
                    {
                        candidates_.insert(std::make_pair(
                            absSq(prefVelocity_ - candidate.position_), candidate));
                    }
                }
            }

            d = det(velocityObstacles_[i].side1_, velocityObstacles_[j].side2_);

            if (d != 0.0f)
            {
                const float s =
                    det(velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_,
                        velocityObstacles_[j].side2_) /
                    d;
                const float t =
                    det(velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_,
                        velocityObstacles_[i].side1_) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate.position_ =
                        velocityObstacles_[i].apex_ + s * velocityObstacles_[i].side1_;

                    if (absSq(candidate.position_) < maxSpeed_ * maxSpeed_)
                    {
                        candidates_.insert(std::make_pair(
                            absSq(prefVelocity_ - candidate.position_), candidate));
                    }
                }
            }

            d = det(velocityObstacles_[i].side2_, velocityObstacles_[j].side2_);

            if (d != 0.0f)
            {
                const float s =
                    det(velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_,
                        velocityObstacles_[j].side2_) /
                    d;
                const float t =
                    det(velocityObstacles_[j].apex_ - velocityObstacles_[i].apex_,
                        velocityObstacles_[i].side2_) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate.position_ =
                        velocityObstacles_[i].apex_ + s * velocityObstacles_[i].side2_;

                    if (absSq(candidate.position_) < maxSpeed_ * maxSpeed_)
                    {
                        candidates_.insert(std::make_pair(
                            absSq(prefVelocity_ - candidate.position_), candidate));
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
                det(velocityObstacles_[j].side2_,
                    candidate.position_ - velocityObstacles_[j].apex_) < 0.0f &&
                det(velocityObstacles_[j].side1_,
                    candidate.position_ - velocityObstacles_[j].apex_) > 0.0f)
            {
                valid = false;

                if (j > optimal)
                {
                    optimal      = j;
                    newVelocity_ = candidate.position_;
                }

                break;
            }
        }

        if (valid)
        {
            newVelocity_ = candidate.position_;
            break;
        }
    }
}

void HRVOAgent::computePreferredVelocity()
{
    if (prefSpeed_ <= 0.01f || maxAccel_ <= 0.01f)
    {
        prefVelocity_ = Vector2(0.f, 0.f);
        return;
    }

    // TODO (#2374): Update so we have the same logic for when the robot is accelerating
    // https://github.com/UBC-Thunderbots/Software/issues/2374
    std::unique_ptr<Goal> &nextGoal = simulator_->goals_[goalNo_];
    Vector2 goalPosition            = nextGoal->getCurrentGoalPosition();
    float speedAtGoal               = nextGoal->getDesiredSpeedAtCurrentGoal();
    Vector2 distVectorToGoal        = goalPosition - position_;
    auto distToGoal = static_cast<float>(std::sqrt(std::pow(distVectorToGoal.getX(), 2) +
                                                   std::pow(distVectorToGoal.getY(), 2)));
    // d = (Vf^2 - Vi^2) / 2a
    double startLinearDecelerationDistance =
        std::abs((std::pow(speedAtGoal, 2) - std::pow(prefSpeed_, 2)) / (2 * maxAccel_));

    if (distToGoal < startLinearDecelerationDistance)
    {
        // velocity given linear deceleration, distance away from goal, and desired final
        // speed
        auto currPrefSpeed = static_cast<float>(
            std::sqrt(std::pow(speedAtGoal, 2) + 2 * maxAccel_ * distToGoal));
        prefVelocity_ = normalize(distVectorToGoal) * currPrefSpeed;
    }
    else
    {
        prefVelocity_ = normalize(goalPosition - position_) * prefSpeed_;
    }
}

void HRVOAgent::insertNeighbor(std::size_t agentNo, float &rangeSq)
{
    const std::unique_ptr<Agent> &other = simulator_->agents_[agentNo];

    if (this != other.get())
    {
        const float distSq = absSq(position_ - other->position_);

        if (distSq < std::pow(radius_ + other->radius_, 2) && distSq < rangeSq)
        {
            neighbors_.clear();

            if (neighbors_.size() == maxNeighbors_)
            {
                neighbors_.erase(--neighbors_.end());
            }

            neighbors_.insert(std::make_pair(distSq, agentNo));

            if (neighbors_.size() == maxNeighbors_)
            {
                rangeSq = (--neighbors_.end())->first;
            }
        }
        else if (distSq < rangeSq)
        {
            if (neighbors_.size() == maxNeighbors_)
            {
                neighbors_.erase(--neighbors_.end());
            }

            neighbors_.insert(std::make_pair(distSq, agentNo));

            if (neighbors_.size() == maxNeighbors_)
            {
                rangeSq = (--neighbors_.end())->first;
            }
        }
    }
}

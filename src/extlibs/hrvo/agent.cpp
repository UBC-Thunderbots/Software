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

#include "agent.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include "extlibs/hrvo/goal.h"
#include "extlibs/hrvo/kd_tree.h"

Agent::Agent(Simulator *simulator)
    : simulator_(simulator),
      goalNo_(0),
      maxNeighbors_(0),
      goalRadius_(0.0f),
      maxAccel_(0.0f),
      maxSpeed_(0.0f),
      neighborDist_(0.0f),
      orientation_(0.0f),
      prefSpeed_(0.0f),
      radius_(0.0f),
      uncertaintyOffset_(0.0f),
      reachedGoal_(false)
{
}

Agent::Agent(Simulator *simulator, const Vector &position, std::size_t goalNo)
    : simulator_(simulator),
      newVelocity_(simulator_->defaults_->velocity_),
      position_(position),
      velocity_(simulator_->defaults_->velocity_),
      goalNo_(goalNo),
      maxNeighbors_(simulator_->defaults_->maxNeighbors_),
      goalRadius_(simulator_->defaults_->goalRadius_),
      maxAccel_(simulator_->defaults_->maxAccel_),
      maxSpeed_(simulator_->defaults_->maxSpeed_),
      neighborDist_(simulator_->defaults_->neighborDist_),
      orientation_(simulator_->defaults_->orientation_),
      prefSpeed_(simulator_->defaults_->prefSpeed_),
      radius_(simulator_->defaults_->radius_),
      uncertaintyOffset_(simulator_->defaults_->uncertaintyOffset_),
      reachedGoal_(false)
{
}

Agent::Agent(Simulator *simulator, const Vector &position, std::size_t goalNo,
             float neighborDist, std::size_t maxNeighbors, float radius,
             const Vector &velocity, float maxAccel, float goalRadius, float prefSpeed,
             float maxSpeed, float orientation, float uncertaintyOffset)
    : simulator_(simulator),
      newVelocity_(velocity),
      position_(position),
      velocity_(velocity),
      goalNo_(goalNo),
      maxNeighbors_(maxNeighbors),
      goalRadius_(goalRadius),
      maxAccel_(maxAccel),
      maxSpeed_(maxSpeed),
      neighborDist_(neighborDist),
      orientation_(orientation),
      prefSpeed_(prefSpeed),
      radius_(radius),
      uncertaintyOffset_(uncertaintyOffset),
      reachedGoal_(false)
{
}

void Agent::computeNeighbors()
{
    neighbors_.clear();
    simulator_->kdTree_->query(this, neighborDist_ * neighborDist_);
}

void Agent::computeNewVelocity()
{
    velocityObstacles_.clear();
    velocityObstacles_.reserve(neighbors_.size());

    VelocityObstacle velocityObstacle;

    for (std::set<std::pair<float, std::size_t>>::const_iterator iter =
             neighbors_.begin();
         iter != neighbors_.end(); ++iter)
    {
        const Agent *const other = simulator_->agents_[iter->second];

        if ((other->position_ - position_).lengthSquared() >
            std::pow(other->radius_ + radius_, 2))
        {
            const float angle        = atan2((other->position_ - position_).y(),
                                      (other->position_ - position_).x());
            const float openingAngle = std::asin((other->radius_ + radius_) /
                                                 (other->position_ - position_).length());

            velocityObstacle.side1_ =
                Vector(std::cos(angle - openingAngle), std::sin(angle - openingAngle));
            velocityObstacle.side2_ =
                Vector(std::cos(angle + openingAngle), std::sin(angle + openingAngle));

            const float d = 2.0f * std::sin(openingAngle) * std::cos(openingAngle);

            if ((other->position_ - position_).det(prefVelocity_ - other->prefVelocity_) >
                0.0f)
            {
                const float s =
                    0.5f * (velocity_ - other->velocity_).det(velocityObstacle.side2_) /
                    d;

                velocityObstacle.apex_ =
                    other->velocity_ + s * velocityObstacle.side1_ -
                    (uncertaintyOffset_ * (other->position_ - position_).length() /
                     (other->radius_ + radius_)) *
                        (other->position_ - position_).normalize();
            }
            else
            {
                const float s =
                    0.5f * (velocity_ - other->velocity_).det(velocityObstacle.side1_) /
                    d;

                velocityObstacle.apex_ =
                    other->velocity_ + s * velocityObstacle.side2_ -
                    (uncertaintyOffset_ * (other->position_ - position_).length() /
                     (other->radius_ + radius_)) *
                        (other->position_ - position_).normalize();
            }

            velocityObstacles_.push_back(velocityObstacle);
        }
        else
        {
            velocityObstacle.apex_ =
                0.5f * (other->velocity_ + velocity_) -
                (uncertaintyOffset_ + 0.5f *
                                          (other->radius_ + radius_ -
                                           (other->position_ - position_).length()) /
                                          simulator_->timeStep_) *
                    (other->position_ - position_).normalize();
            velocityObstacle.side1_ = (position_, other->position_).perpendicular();
            velocityObstacle.side2_ = -velocityObstacle.side1_;
            velocityObstacles_.push_back(velocityObstacle);
        }
    }

    candidates_.clear();

    Candidate candidate;

    candidate.velocityObstacle1_ = std::numeric_limits<int>::max();
    candidate.velocityObstacle2_ = std::numeric_limits<int>::max();

    if ((prefVelocity_).lengthSquared() < maxSpeed_ * maxSpeed_)
    {
        candidate.position_ = prefVelocity_;
    }
    else
    {
        candidate.position_ = maxSpeed_ * (prefVelocity_).normalize();
    }

    candidates_.insert(
        std::make_pair((prefVelocity_ - candidate.position_).lengthSquared(), candidate));

    for (int i = 0; i < static_cast<int>(velocityObstacles_.size()); ++i)
    {
        candidate.velocityObstacle1_ = i;
        candidate.velocityObstacle2_ = i;

        const float dotProduct1 = (prefVelocity_ - velocityObstacles_[i].apex_)
                                      .dot(velocityObstacles_[i].side1_);
        const float dotProduct2 = (prefVelocity_ - velocityObstacles_[i].apex_)
                                      .dot(velocityObstacles_[i].side2_);

        if (dotProduct1 > 0.0f &&
            (velocityObstacles_[i].side1_)
                    .det(prefVelocity_ - velocityObstacles_[i].apex_) > 0.0f)
        {
            candidate.position_ =
                velocityObstacles_[i].apex_ + dotProduct1 * velocityObstacles_[i].side1_;

            if ((candidate.position_).lengthSquared() < maxSpeed_ * maxSpeed_)
            {
                candidates_.insert(std::make_pair(
                    (prefVelocity_ - candidate.position_).lengthSquared(), candidate));
            }
        }

        if (dotProduct2 > 0.0f &&
            (velocityObstacles_[i].side2_)
                    .det(prefVelocity_ - velocityObstacles_[i].apex_) < 0.0f)
        {
            candidate.position_ =
                velocityObstacles_[i].apex_ + dotProduct2 * velocityObstacles_[i].side2_;

            if ((candidate.position_).lengthSquared() < maxSpeed_ * maxSpeed_)
            {
                candidates_.insert(std::make_pair(
                    (prefVelocity_ - candidate.position_).lengthSquared(), candidate));
            }
        }
    }

    for (int j = 0; j < static_cast<int>(velocityObstacles_.size()); ++j)
    {
        candidate.velocityObstacle1_ = std::numeric_limits<int>::max();
        candidate.velocityObstacle2_ = j;

        float discriminant =
            maxSpeed_ * maxSpeed_ -
            std::pow((velocityObstacles_[j].apex_).det(velocityObstacles_[j].side1_),
                     2.f);

        if (discriminant > 0.0f)
        {
            const float t1 =
                -(velocityObstacles_[j].apex_).dot(velocityObstacles_[j].side1_) +
                std::sqrt(discriminant);
            const float t2 =
                -(velocityObstacles_[j].apex_).dot(velocityObstacles_[j].side1_) -
                std::sqrt(discriminant);

            if (t1 >= 0.0f)
            {
                candidate.position_ =
                    velocityObstacles_[j].apex_ + t1 * velocityObstacles_[j].side1_;
                candidates_.insert(std::make_pair(
                    (prefVelocity_ - candidate.position_).lengthSquared(), candidate));
            }

            if (t2 >= 0.0f)
            {
                candidate.position_ =
                    velocityObstacles_[j].apex_ + t2 * velocityObstacles_[j].side1_;
                candidates_.insert(std::make_pair(
                    (prefVelocity_ - candidate.position_).lengthSquared(), candidate));
            }
        }

        discriminant =
            maxSpeed_ * maxSpeed_ -
            std::pow((velocityObstacles_[j].apex_).det(velocityObstacles_[j].side2_),
                     2.f);

        if (discriminant > 0.0f)
        {
            const float t1 =
                -(velocityObstacles_[j].apex_).dot(velocityObstacles_[j].side2_) +
                std::sqrt(discriminant);
            const float t2 =
                -(velocityObstacles_[j].apex_).dot(velocityObstacles_[j].side2_) -
                std::sqrt(discriminant);

            if (t1 >= 0.0f)
            {
                candidate.position_ =
                    velocityObstacles_[j].apex_ + t1 * velocityObstacles_[j].side2_;
                candidates_.insert(std::make_pair(
                    (prefVelocity_ - candidate.position_).lengthSquared(), candidate));
            }

            if (t2 >= 0.0f)
            {
                candidate.position_ =
                    velocityObstacles_[j].apex_ + t2 * velocityObstacles_[j].side2_;
                candidates_.insert(std::make_pair(
                    (prefVelocity_ - candidate.position_).lengthSquared(), candidate));
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

                    if ((candidate.position_).lengthSquared() < maxSpeed_ * maxSpeed_)
                    {
                        candidates_.insert(std::make_pair(
                            (prefVelocity_ - candidate.position_).lengthSquared(),
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

                    if ((candidate.position_).lengthSquared() < maxSpeed_ * maxSpeed_)
                    {
                        candidates_.insert(std::make_pair(
                            (prefVelocity_ - candidate.position_).lengthSquared(),
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

                    if ((candidate.position_).lengthSquared() < maxSpeed_ * maxSpeed_)
                    {
                        candidates_.insert(std::make_pair(
                            (prefVelocity_ - candidate.position_).lengthSquared(),
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

                    if ((candidate.position_).lengthSquared() < maxSpeed_ * maxSpeed_)
                    {
                        candidates_.insert(std::make_pair(
                            (prefVelocity_ - candidate.position_).lengthSquared(),
                            candidate));
                    }
                }
            }
        }
    }

    int optimal = -1;

    for (std::multimap<float, Candidate>::const_iterator iter = candidates_.begin();
         iter != candidates_.end(); ++iter)
    {
        candidate  = iter->second;
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

void Agent::computePreferredVelocity()
{
    if (prefSpeed_ <= 0.1f || maxAccel_ <= 0.1f)
    {
        prefVelocity_ = Vector(0.f, 0.f);
        return;
    }

    // TODO (#2374): Update so we have the same logic for when the robot is accelerating
    // https://github.com/UBC-Thunderbots/Software/issues/2374
    Goal *nextGoal          = simulator_->goals_[goalNo_];
    Vector goalPosition     = nextGoal->getCurrentGoalPosition();
    float speedAtGoal       = nextGoal->getDesiredSpeedAtCurrentGoal();
    Vector distVectorToGoal = goalPosition - position_;
    auto distToGoal         = static_cast<float>(
        std::sqrt(std::pow(distVectorToGoal.x(), 2) + std::pow(distVectorToGoal.y(), 2)));
    // d = (Vf^2 - Vi^2) / 2a
    double startLinearDecelerationDistance =
        std::abs((std::pow(speedAtGoal, 2) - std::pow(prefSpeed_, 2)) / (2 * maxAccel_));

    if (distToGoal < startLinearDecelerationDistance)
    {
        // velocity given linear deceleration, distance away from goal, and desired final
        // speed
        auto currPrefSpeed = static_cast<float>(
            std::sqrt(std::pow(speedAtGoal, 2) + 2 * maxAccel_ * distToGoal));
        prefVelocity_ = (distVectorToGoal).normalize() * currPrefSpeed;
    }
    else
    {
        prefVelocity_ = (goalPosition - position_).normalize() * prefSpeed_;
    }
}

Vector Agent::getVelocity() const
{
    return velocity_;
}

void Agent::insertNeighbor(std::size_t agentNo, float &rangeSq)
{
    const Agent *const other = simulator_->agents_[agentNo];

    if (this != other)
    {
        const float distSq = (position_ - other->position_).lengthSquared();

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

void Agent::update()
{
    const float dv = (newVelocity_ - velocity_).length();

    if (dv < maxAccel_ * simulator_->timeStep_)
    {
        velocity_ = newVelocity_;
    }
    else
    {
        velocity_ = (1.0f - (maxAccel_ * simulator_->timeStep_ / dv)) * velocity_ +
                    (maxAccel_ * simulator_->timeStep_ / dv) * newVelocity_;
    }

    position_ += velocity_ * simulator_->timeStep_;

    if ((simulator_->goals_[goalNo_]->getCurrentGoalPosition() - position_)
            .lengthSquared() < goalRadius_ * goalRadius_)
    {
        // Is at current goal position
        if (simulator_->goals_[goalNo_]->isGoingToFinalGoal())
        {
            reachedGoal_ = true;
        }
        else
        {
            simulator_->goals_[goalNo_]->getNextGoalPostion();
            reachedGoal_              = false;
            simulator_->reachedGoals_ = false;
        }
    }
    else
    {
        reachedGoal_              = false;
        simulator_->reachedGoals_ = false;
    }

    if (!reachedGoal_)
    {
        orientation_ = atan2(prefVelocity_.y(), prefVelocity_.x());
    }
}

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
#include "software/geom/algorithms/rasterize.h"


HRVOAgent::HRVOAgent(HRVOSimulator *simulator, const Vector2 &position,
                     std::size_t goalIndex, float neighborDist, std::size_t maxNeighbors,
                     float radius, const Vector2 &velocity, float maxAccel,
                     float goalRadius, float prefSpeed, float maxSpeed,
                     float uncertaintyOffset)
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

    std::unique_ptr<Goal> &current_goal = simulator_->goals[goal_index_];
    float new_neighbor_dist =
        std::min(neighborDist_,
                 abs(position_ - current_goal->getCurrentGoalPosition()) + goal_radius_);

    simulator_->getKdTree()->query(this, new_neighbor_dist);
}

Agent::VelocityObstacle HRVOAgent::createVelocityObstacle(const Agent &other_agent)
{
    VelocityObstacle velocityObstacle;
    if (absSq(position_ - other_agent.getPosition()) >
        std::pow(radius_ + other_agent.getRadius(), 2))
    {
        // This Agent is not colliding with other agent
        const float angle = atan(position_ - other_agent.getPosition());

        // The opening angle of the velocity obstacle
        // opening angle = arcsin((rad_A + rad_B) / distance_BA)
        const float openingAngle = std::asin((radius_ + other_agent.getRadius()) /
                                             abs(position_ - other_agent.getPosition()));

        // Direction of the two edges of the velocity obstacle
        velocityObstacle.side1_ =
            Vector2(std::cos(angle - openingAngle), std::sin(angle - openingAngle));
        velocityObstacle.side2_ =
            Vector2(std::cos(angle + openingAngle), std::sin(angle + openingAngle));

        const float d = std::sin(2.f * openingAngle);

        // This shifts one side of the velocity obstacle to share the responsibility
        // of avoiding collision with other agent. This assumes that other agent will also
        // be running HRVO
        if (det(position_ - other_agent.getPosition(),
                other_agent.getPrefVelocity() - pref_velocity_) > 0.0f)
        {
            // Relative velocity is in the right half of velocity obstacle (VO)
            // Shift the VO apex to the left so the right side is smaller, making the
            // VO a Hybrid Reciprocal Velocity Obstacle (HRVO)
            const float s =
                0.5f *
                det(other_agent.getVelocity() - velocity_, velocityObstacle.side2_) / d;

            velocityObstacle.apex_ =
                velocity_ + s * velocityObstacle.side1_ -
                (uncertaintyOffset_ * abs(position_ - other_agent.getPosition()) /
                 (radius_ + other_agent.getRadius())) *
                    normalize(position_ - other_agent.getPosition());
        }
        else
        {
            // Relative velocity is in the left half of velocity obstacle (VO)
            // Shift the VO apex to the right so the left side is smaller, making the
            // VO a Hybrid Reciprocal Velocity Obstacle (HRVO)
            const float s =
                0.5f *
                det(other_agent.getVelocity() - velocity_, velocityObstacle.side1_) / d;

            velocityObstacle.apex_ =
                velocity_ + s * velocityObstacle.side2_ -
                (uncertaintyOffset_ * abs(position_ - other_agent.getPosition()) /
                 (other_agent.getRadius() + radius_)) *
                    normalize(position_ - other_agent.getPosition());
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
                                       abs(position_ - other_agent.getPosition())) /
                                      simulator_->getTimeStep()) *
                normalize(position_ - other_agent.getPosition());
        velocityObstacle.side1_ = normal(other_agent.getPosition(), position_);
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
        std::shared_ptr<Agent> other_agent = simulator_->getAgents()[neighbor.second];
        VelocityObstacle velocity_obstacle = other_agent->createVelocityObstacle(*this);
        velocityObstacles_.push_back(velocity_obstacle);
    }

    candidates_.clear();
    Candidate candidate;

    candidate.velocityObstacle1_ = std::numeric_limits<int>::max();
    candidate.velocityObstacle2_ = std::numeric_limits<int>::max();

    if (absSq(pref_velocity_) < max_speed_ * max_speed_)
    {
        candidate.position_ = pref_velocity_;
    }
    else
    {
        candidate.position_ = max_speed_ * normalize(pref_velocity_);
    }

    candidates_.insert(
        std::make_pair(absSq(pref_velocity_ - candidate.position_), candidate));

    for (int i = 0; i < static_cast<int>(velocityObstacles_.size()); ++i)
    {
        candidate.velocityObstacle1_ = i;
        candidate.velocityObstacle2_ = i;

        const float dotProduct1 =
            (pref_velocity_ - velocityObstacles_[i].apex_) * velocityObstacles_[i].side1_;
        const float dotProduct2 =
            (pref_velocity_ - velocityObstacles_[i].apex_) * velocityObstacles_[i].side2_;

        if (dotProduct1 > 0.0f &&
            det(velocityObstacles_[i].side1_,
                pref_velocity_ - velocityObstacles_[i].apex_) > 0.0f)
        {
            candidate.position_ =
                velocityObstacles_[i].apex_ + dotProduct1 * velocityObstacles_[i].side1_;

            if (absSq(candidate.position_) < max_speed_ * max_speed_)
            {
                candidates_.insert(std::make_pair(
                    absSq(pref_velocity_ - candidate.position_), candidate));
            }
        }

        if (dotProduct2 > 0.0f &&
            det(velocityObstacles_[i].side2_,
                pref_velocity_ - velocityObstacles_[i].apex_) < 0.0f)
        {
            candidate.position_ =
                velocityObstacles_[i].apex_ + dotProduct2 * velocityObstacles_[i].side2_;

            if (absSq(candidate.position_) < max_speed_ * max_speed_)
            {
                candidates_.insert(std::make_pair(
                    absSq(pref_velocity_ - candidate.position_), candidate));
            }
        }
    }

    for (int j = 0; j < static_cast<int>(velocityObstacles_.size()); ++j)
    {
        candidate.velocityObstacle1_ = std::numeric_limits<int>::max();
        candidate.velocityObstacle2_ = j;

        float discriminant =
            max_speed_ * max_speed_ -
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
                    absSq(pref_velocity_ - candidate.position_), candidate));
            }

            if (t2 >= 0.0f)
            {
                candidate.position_ =
                    velocityObstacles_[j].apex_ + t2 * velocityObstacles_[j].side1_;
                candidates_.insert(std::make_pair(
                    absSq(pref_velocity_ - candidate.position_), candidate));
            }
        }

        discriminant =
            max_speed_ * max_speed_ -
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
                    absSq(pref_velocity_ - candidate.position_), candidate));
            }

            if (t2 >= 0.0f)
            {
                candidate.position_ =
                    velocityObstacles_[j].apex_ + t2 * velocityObstacles_[j].side2_;
                candidates_.insert(std::make_pair(
                    absSq(pref_velocity_ - candidate.position_), candidate));
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

                    if (absSq(candidate.position_) < max_speed_ * max_speed_)
                    {
                        candidates_.insert(std::make_pair(
                            absSq(pref_velocity_ - candidate.position_), candidate));
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

                    if (absSq(candidate.position_) < max_speed_ * max_speed_)
                    {
                        candidates_.insert(std::make_pair(
                            absSq(pref_velocity_ - candidate.position_), candidate));
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

                    if (absSq(candidate.position_) < max_speed_ * max_speed_)
                    {
                        candidates_.insert(std::make_pair(
                            absSq(pref_velocity_ - candidate.position_), candidate));
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

                    if (absSq(candidate.position_) < max_speed_ * max_speed_)
                    {
                        candidates_.insert(std::make_pair(
                            absSq(pref_velocity_ - candidate.position_), candidate));
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
        pref_velocity_ = Vector2(0.f, 0.f);
        return;
    }

    std::unique_ptr<Goal> &nextGoal = simulator_->goals[goal_index_];
    Vector2 goalPosition            = nextGoal->getCurrentGoalPosition();
    float speedAtGoal               = nextGoal->getDesiredSpeedAtCurrentGoal();
    Vector2 distVectorToGoal        = goalPosition - position_;
    auto distToGoal = static_cast<float>(std::sqrt(std::pow(distVectorToGoal.getX(), 2) +
                                                   std::pow(distVectorToGoal.getY(), 2)));
    // d = (Vf^2 - Vi^2) / 2a
    double startLinearDecelerationDistance =
        std::abs((std::pow(speedAtGoal, 2) - std::pow(prefSpeed_, 2)) / (2 * max_accel_)) * 1.02f; // TODO: Tune constant

    if (distToGoal < startLinearDecelerationDistance)
    {
        // velocity given linear deceleration, distance away from goal, and desired final
        // speed
        // v_pref = sqrt(v_goal^2 + 2 * a * d_remainingToDestination)
        float currPrefSpeed = static_cast<float>(
            std::sqrt(std::pow(speedAtGoal, 2) + 2 * max_accel_ * distToGoal)) * 0.98f; // TODO: Tune constant
        Vector2 ideal_pref_velocity = normalize(distVectorToGoal) * currPrefSpeed;

        // Limit the preferred velocity to the kinematic limits
        const Vector2 dv = ideal_pref_velocity - velocity_;
        if (abs(dv) <= max_accel_ * simulator_->getTimeStep())
        {
            pref_velocity_ = ideal_pref_velocity;
        }
        else
        {
            // Calculate the maximum velocity towards the preferred velocity, given the
            // acceleration constraint
            pref_velocity_ =
                velocity_ + (max_accel_ * simulator_->getTimeStep()) * (dv / abs(dv));
        }
    }
    else
    {
        // Accelerate to preferred speed
        // v_pref = v_now + a * t
        float currPrefSpeed =
            std::min(prefSpeed_, abs(velocity_) + max_accel_ * simulator_->getTimeStep());
        pref_velocity_ = normalize(distVectorToGoal) * currPrefSpeed;
    }
}

void HRVOAgent::insertNeighbor(std::size_t agentNo, float &rangeSq)
{
    std::shared_ptr<Agent> other_agent = simulator_->getAgents()[agentNo];

    if (this != other_agent.get())
    {
        Vector2 other_agent_relative_pos = other_agent->getPosition() - position_;
        const float distSq               = absSq(other_agent_relative_pos);

        Vector2 goal_pos = simulator_->goals[goal_index_]->getCurrentGoalPosition();
        Vector2 relative_goal_pos = goal_pos - position_;

        // Whether the other robot is with in 45 degrees of the goal, relative to us
        const float forty_five_deg_ratio = 1.f / std::sqrt(2.f);
        bool is_other_agent_in_front =
            dotProduct(normalize(relative_goal_pos),
                       normalize(other_agent_relative_pos)) > forty_five_deg_ratio;

        bool is_other_agent_moving_towards_us =
            dotProduct(normalize(velocity_), normalize(other_agent->getVelocity())) <
            -forty_five_deg_ratio;

        // Whether the other agent is within a 1.5-meter radius of our goal
        bool is_other_agent_near_goal =
            absSq(other_agent->getPosition() - goal_pos) < 1.5f;

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
}

std::vector<Polygon> HRVOAgent::getVelocityObstaclesAsPolygons() const
{
    std::vector<Polygon> velocity_obstacles;
    for (const Agent::VelocityObstacle &vo : velocityObstacles_)
    {
        std::vector<Point> points;
        Vector2 shifted_apex  = position_ + vo.apex_;
        Vector2 shifted_side1 = position_ + vo.side1_;
        Vector2 shifted_side2 = position_ + vo.side2_;
        points.emplace_back(Point(shifted_apex.getX(), shifted_apex.getY()));
        points.emplace_back(Point(shifted_side1.getX(), shifted_side1.getY()));
        points.emplace_back(Point(shifted_side2.getX(), shifted_side2.getY()));
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
        Vector2 candidate_pos = position_ + candidate.second.position_;
        candidate_circles.emplace_back(
            Circle(Point(candidate_pos.getX(), candidate_pos.getY()), circle_rad));
    }
    return candidate_circles;
}

void HRVOAgent::setPreferredSpeed(float new_pref_speed)
{
    prefSpeed_ = new_pref_speed;
}

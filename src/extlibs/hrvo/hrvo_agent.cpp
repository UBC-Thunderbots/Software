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
#include <limits>

#include "path.h"
#include "proto/message_translation/tbots_protobuf.h"
#include "software/geom/vector.h"
#include "software/logger/logger.h"


HRVOAgent::HRVOAgent(HRVOSimulator *simulator, const Vector &position,
                     const Vector &velocity, float pref_speed, float max_speed,
                     float max_accel, AgentPath &path, float radius,
                     std::size_t max_num_neighbors, float max_neighbor_dist,
                     float uncertainty_offset)
    : Agent(simulator, position, velocity, velocity, max_speed, max_accel, path, radius),
      max_neighbors_(max_num_neighbors),
      neighbor_dist_(max_neighbor_dist),
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

Agent::VelocityObstacle HRVOAgent::createVelocityObstacle(const Agent &other_agent)
{
    VelocityObstacle velocity_obstacle;
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
        velocity_obstacle.side1_ =
            Vector::createFromAngle(Angle::fromRadians(angle - opening_angle));
        velocity_obstacle.side2_ =
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
            const float s = 0.5f *
                            (other_agent.getVelocity() - velocity_)
                                .determinant(velocity_obstacle.side2_) /
                            d;

            velocity_obstacle.apex_ =
                velocity_ + s * velocity_obstacle.side1_ -
                (position_ - other_agent.getPosition())
                    .normalize((uncertainty_offset_ *
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
                                .determinant(velocity_obstacle.side1_) /
                            d;

            velocity_obstacle.apex_ =
                velocity_ + s * velocity_obstacle.side2_ -
                (position_ - other_agent.getPosition())
                    .normalize(uncertainty_offset_ *
                               (position_ - other_agent.getPosition()).length() /
                               (other_agent.getRadius() + radius_));
        }
    }
    else
    {
        // This Agent is colliding with other agent
        // Uses Reciprocal Velocity Obstacle (RVO) with the sides being 180 degrees
        // apart from each other
        velocity_obstacle.apex_ =
            0.5f * (other_agent.getVelocity() + velocity_) -
            (position_ - other_agent.getPosition())
                .normalize(uncertainty_offset_ +
                           0.5f *
                               (other_agent.getRadius() + radius_ -
                                (position_ - other_agent.getPosition()).length()) /
                               simulator_->getTimeStep());
        velocity_obstacle.side1_ =
            (other_agent.getPosition() - position_).perpendicular().normalize();
        velocity_obstacle.side2_ = -velocity_obstacle.side1_;
    }

    return velocity_obstacle;
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
    candidate.velocity_obstacle1_ = std::numeric_limits<int>::max();
    candidate.velocity_obstacle2_ = std::numeric_limits<int>::max();

    if (pref_velocity_.lengthSquared() < max_speed_ * max_speed_)
    {
        candidate.position_ = pref_velocity_;
    }
    else
    {
        candidate.position_ = pref_velocity_.normalize(max_speed_);
    }

    candidates_.insert(std::make_pair(
        (pref_velocity_ - candidate.position_).lengthSquared(), candidate));

    // The ones close to the robot when stationary
    for (int i = 0; i < static_cast<int>(velocity_obstacles_.size()); ++i)
    {
        candidate.velocity_obstacle1_ = i;
        candidate.velocity_obstacle2_ = i;

        const float dot_product_1 = (pref_velocity_ - velocity_obstacles_[i].apex_)
                                        .dot(velocity_obstacles_[i].side1_);
        const float dot_product_2 = (pref_velocity_ - velocity_obstacles_[i].apex_)
                                        .dot(velocity_obstacles_[i].side2_);

        if (dot_product_1 > 0.0f &&
            (velocity_obstacles_[i].side1_)
                    .determinant(pref_velocity_ - velocity_obstacles_[i].apex_) > 0.0f)
        {
            candidate.position_ = velocity_obstacles_[i].apex_ +
                                  dot_product_1 * velocity_obstacles_[i].side1_;

            if (candidate.position_.lengthSquared() < max_speed_ * max_speed_)
            {
                candidates_.insert(std::make_pair(
                    (pref_velocity_ - candidate.position_).lengthSquared(), candidate));
            }
        }

        if (dot_product_2 > 0.0f &&
            (velocity_obstacles_[i].side2_)
                    .determinant(pref_velocity_ - velocity_obstacles_[i].apex_) < 0.0f)
        {
            candidate.position_ = velocity_obstacles_[i].apex_ +
                                  dot_product_2 * velocity_obstacles_[i].side2_;

            if (candidate.position_.lengthSquared() < max_speed_ * max_speed_)
            {
                candidates_.insert(std::make_pair(
                    (pref_velocity_ - candidate.position_).lengthSquared(), candidate));
            }
        }
    }

     // The ones far away from the robot when stationary TODO: Convert naming? or remove
        for (int j = 0; j < static_cast<int>(velocity_obstacles_.size()); ++j)
        {
            candidate.velocity_obstacle1_ = std::numeric_limits<int>::max();
            candidate.velocity_obstacle2_ = j;

            float discriminant =
                max_speed_ * max_speed_ -
                std::pow(
                    (velocity_obstacles_[j].apex_).determinant(velocity_obstacles_[j].side1_),
                    2.f);

            if (discriminant > 0.0f)
            {
                const float t1 =
                    -(velocity_obstacles_[j].apex_.dot(velocity_obstacles_[j].side1_)) +
                    std::sqrt(discriminant);
                const float t2 =
                    -(velocity_obstacles_[j].apex_.dot(velocity_obstacles_[j].side1_)) -
                    std::sqrt(discriminant);

                if (t1 >= 0.0f)
                {
                    candidate.position_ =
                        velocity_obstacles_[j].apex_ + t1 *
                        velocity_obstacles_[j].side1_;
                    candidates_.insert(std::make_pair(
                        (pref_velocity_ - candidate.position_).lengthSquared(),
                        candidate));
                }

                if (t2 >= 0.0f)
                {
                    candidate.position_ =
                        velocity_obstacles_[j].apex_ + t2 *
                        velocity_obstacles_[j].side1_;
                    candidates_.insert(std::make_pair(
                        (pref_velocity_ - candidate.position_).lengthSquared(),
                        candidate));
                }
            }

            discriminant =
                max_speed_ * max_speed_ -
                std::pow(
                    (velocity_obstacles_[j].apex_).determinant(velocity_obstacles_[j].side2_),
                    2.f);

            if (discriminant > 0.0f)
            {
                const float t1 =
                    -(velocity_obstacles_[j].apex_.dot(velocity_obstacles_[j].side2_)) +
                    std::sqrt(discriminant);
                const float t2 =
                    -(velocity_obstacles_[j].apex_.dot(velocity_obstacles_[j].side2_)) -
                    std::sqrt(discriminant);

                if (t1 >= 0.0f)
                {
                    candidate.position_ =
                        velocity_obstacles_[j].apex_ + t1 *
                        velocity_obstacles_[j].side2_;
                    candidates_.insert(std::make_pair(
                        (pref_velocity_ - candidate.position_).lengthSquared(),
                        candidate));
                }

                if (t2 >= 0.0f)
                {
                    candidate.position_ =
                        velocity_obstacles_[j].apex_ + t2 *
                        velocity_obstacles_[j].side2_;
                    candidates_.insert(std::make_pair(
                        (pref_velocity_ - candidate.position_).lengthSquared(),
                        candidate));
                }
            }
        }

     // The one right on top of the robot
        for (int i = 0; i < static_cast<int>(velocity_obstacles_.size()) - 1; ++i)
        {
            for (int j = i + 1; j < static_cast<int>(velocity_obstacles_.size()); ++j)
            {
                candidate.velocity_obstacle1_ = i;
                candidate.velocity_obstacle2_ = j;

                float d =
                    (velocity_obstacles_[i].side1_).determinant(velocity_obstacles_[j].side1_);

                if (d != 0.0f)
                {
                    const float s =
                        (velocity_obstacles_[j].apex_ - velocity_obstacles_[i].apex_)
                            .determinant(velocity_obstacles_[j].side1_) /
                        d;
                    const float t =
                        (velocity_obstacles_[j].apex_ - velocity_obstacles_[i].apex_)
                            .determinant(velocity_obstacles_[i].side1_) /
                        d;

                    if (s >= 0.0f && t >= 0.0f)
                    {
                        candidate.position_ =
                            velocity_obstacles_[i].apex_ + s *
                            velocity_obstacles_[i].side1_;

                        if (candidate.position_.lengthSquared() < max_speed_ *
                        max_speed_)
                        {
                            candidates_.insert(std::make_pair(
                                (pref_velocity_ - candidate.position_).lengthSquared(),
                                candidate));
                        }
                    }
                }

                d =
                (velocity_obstacles_[i].side2_).determinant(velocity_obstacles_[j].side1_);

                if (d != 0.0f)
                {
                    const float s =
                        (velocity_obstacles_[j].apex_ - velocity_obstacles_[i].apex_)
                            .determinant(velocity_obstacles_[j].side1_) /
                        d;
                    const float t =
                        (velocity_obstacles_[j].apex_ - velocity_obstacles_[i].apex_)
                            .determinant(velocity_obstacles_[i].side2_) /
                        d;

                    if (s >= 0.0f && t >= 0.0f)
                    {
                        candidate.position_ =
                            velocity_obstacles_[i].apex_ + s *
                            velocity_obstacles_[i].side2_;

                        if (candidate.position_.lengthSquared() < max_speed_ *
                        max_speed_)
                        {
                            candidates_.insert(std::make_pair(
                                (pref_velocity_ - candidate.position_).lengthSquared(),
                                candidate));
                        }
                    }
                }

                d =
                (velocity_obstacles_[i].side1_).determinant(velocity_obstacles_[j].side2_);

                if (d != 0.0f)
                {
                    const float s =
                        (velocity_obstacles_[j].apex_ - velocity_obstacles_[i].apex_)
                            .determinant(velocity_obstacles_[j].side2_) /
                        d;
                    const float t =
                        (velocity_obstacles_[j].apex_ - velocity_obstacles_[i].apex_)
                            .determinant(velocity_obstacles_[i].side1_) /
                        d;

                    if (s >= 0.0f && t >= 0.0f)
                    {
                        candidate.position_ =
                            velocity_obstacles_[i].apex_ + s *
                            velocity_obstacles_[i].side1_;

                        if (candidate.position_.lengthSquared() < max_speed_ *
                        max_speed_)
                        {
                            candidates_.insert(std::make_pair(
                                (pref_velocity_ - candidate.position_).lengthSquared(),
                                candidate));
                        }
                    }
                }

                d =
                (velocity_obstacles_[i].side2_).determinant(velocity_obstacles_[j].side2_);

                if (d != 0.0f)
                {
                    const float s =
                        (velocity_obstacles_[j].apex_ - velocity_obstacles_[i].apex_)
                            .determinant(velocity_obstacles_[j].side2_) /
                        d;
                    const float t =
                        (velocity_obstacles_[j].apex_ - velocity_obstacles_[i].apex_)
                            .determinant(velocity_obstacles_[i].side2_) /
                        d;

                    if (s >= 0.0f && t >= 0.0f)
                    {
                        candidate.position_ =
                            velocity_obstacles_[i].apex_ + s *
                            velocity_obstacles_[i].side2_;

                        if (candidate.position_.lengthSquared() < max_speed_ *
                        max_speed_)
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

        for (int j = 0; j < static_cast<int>(velocity_obstacles_.size()); ++j)
        {
            if (j != candidate.velocity_obstacle1_ &&
                j != candidate.velocity_obstacle2_ &&
                (velocity_obstacles_[j].side2_)
                        .determinant(candidate.position_ - velocity_obstacles_[j].apex_) <
                    0.0f &&
                (velocity_obstacles_[j].side1_)
                        .determinant(candidate.position_ - velocity_obstacles_[j].apex_) >
                    0.0f)
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
    auto path_point_opt = path.getCurrentPathPoint();
    if (pref_speed_ <= 0.01f || max_accel_ <= 0.01f || path_point_opt == std::nullopt)
    {
        // Used to avoid edge cases with division by zero
        pref_velocity_ = Vector(0.f, 0.f);
        return;
    }

    Vector goal_position = path_point_opt.value().getPosition();
    float speed_at_dest  = path_point_opt.value().getSpeed();

    float max_dist_per_tick = max_accel_ * simulator_->getTimeStep();
    Vector vector_to_dest   = goal_position - position_;
    double distance_to_dest = vector_to_dest.length();
    if (distance_to_dest > max_dist_per_tick)
    {
        // Given the lag of HRVO simulator compared to real life, we assume that the robot
        // has travelled the maximum distance for one tick.
        distance_to_dest -= max_dist_per_tick;
    }

    // d = (Vf^2 - Vi^2) / 2a
    double start_linear_deceleration_distance =
        std::abs((std::pow(speed_at_dest, 2) -
                  std::pow(pref_speed_, 2)) /  // TODO: Change to velocity_.length()
                 (2 * max_accel_));

    // TODO: DEBUGGING
    decel_dist = start_linear_deceleration_distance;
    dist_remaining_to_goal = distance_to_dest;

    if (distance_to_dest < start_linear_deceleration_distance)
    {
        // velocity given linear deceleration, distance away from goal, and desired final
        // speed.           + here since a is negative
        // Vi = sqrt(Vf^2 + 2 * a * d)
        double curr_pref_speed =
            std::sqrt(std::pow(speed_at_dest, 2) + 2 * max_accel_ * distance_to_dest);
        Vector ideal_pref_velocity = vector_to_dest.normalize(curr_pref_speed);

        // TODO: DEBUGGING
        ideal_speed = curr_pref_speed;

        // Limit the preferred velocity to the kinematic limits
        const Vector dv = ideal_pref_velocity - velocity_;
        if (dv.length() <= max_dist_per_tick)
        {
            pref_velocity_ = ideal_pref_velocity;
        }
        else
        {
            // Calculate the maximum velocity towards the preferred velocity, given the
            // acceleration constraint
            pref_velocity_ = velocity_ + dv.normalize(max_dist_per_tick);
        }
    }
    else
    {
        // Accelerate to preferred speed
        // Vf = Vi + a * curr_pref_speed
        double curr_pref_speed = std::min(static_cast<double>(pref_speed_),
                                          velocity_.length() + max_dist_per_tick);
        pref_velocity_         = vector_to_dest.normalize(curr_pref_speed);
    }
}

void HRVOAgent::insertNeighbor(std::size_t agent_id, float range_squared)
{
    std::shared_ptr<Agent> other_agent = simulator_->getAgents()[agent_id];
    auto path_point_opt                = path.getCurrentPathPoint();

    if (path_point_opt == std::nullopt || this == other_agent.get())
    {
        return;
    }

    Vector other_agent_relative_pos = other_agent->getPosition() - position_;
    const float dist_squared        = other_agent_relative_pos.lengthSquared();

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

        neighbors_.insert(std::make_pair(dist_squared, agent_id));

        if (neighbors_.size() == max_neighbors_)
        {
            range_squared = (--neighbors_.end())->first;
        }
    };

    if (dist_squared < std::pow(radius_ + other_agent->getRadius(), 2))
    {
        // In collision with other agent, so the other neighbors are not important
        neighbors_.clear();
        add_other_agent();
    }
    else if (dist_squared < range_squared)
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
    for (const Agent::VelocityObstacle &vo : velocity_obstacles_)
    {
        std::vector<Point> points;
        Vector shifted_apex  = position_ + vo.apex_;
        Vector shifted_side1 = position_ + vo.side1_;
        Vector shifted_side2 = position_ + vo.side2_;
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
        Vector candidate_pos = position_ + candidate.second.position_;
        candidate_circles.emplace_back(Circle(Point(candidate_pos), circle_rad));
    }
    return candidate_circles;
}

void HRVOAgent::setPreferredSpeed(float new_pref_speed)
{
    pref_speed_ = new_pref_speed;
}

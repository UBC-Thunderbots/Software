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
#include "proto/message_translation/tbots_geometry.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/vector.h"

HRVOAgent::HRVOAgent(HRVOSimulator *simulator, const Vector &position,
                     float neighbor_dist, std::size_t max_neighbors, float radius,
                     float max_radius_inflation, const Vector &velocity, float max_accel,
                     AgentPath &path, float max_speed)
    : Agent(simulator, position, radius, max_radius_inflation, velocity, velocity,
            max_speed, max_accel, path),
      max_neighbors_(max_neighbors),
      max_neighbor_dist(neighbor_dist),
      pref_speed_(max_speed_ * PREF_SPEED_SCALE),
      // TODO (#2676): This config should be dependency injected and updated when a
      // parameter is changed
      obstacle_factory(TbotsProto::RobotNavigationObstacleConfig()),
      ball_obstacle(std::nullopt)
{
}

void HRVOAgent::updatePrimitive(const TbotsProto::Primitive &new_primitive,
                                const World &world)
{
    AgentPath path;
    static_obstacles.clear();
    ball_obstacle = std::nullopt;
    if (new_primitive.has_move())
    {
        const auto &motion_control = new_primitive.move().motion_control();
        float speed_at_dest        = new_primitive.move().final_speed_m_per_s();
        float new_max_speed        = new_primitive.move().max_speed_m_per_s();
        setMaxSpeed(new_max_speed);
        setPreferredSpeed(new_max_speed * PREF_SPEED_SCALE);

        // TODO (#2418): Update implementation of Primitive to support
        // multiple path points and remove this check
        CHECK(motion_control.path().points().size() >= 2)
            << "Empty path: " << motion_control.path().points().size() << std::endl;
        auto destination = motion_control.path().points().at(1);

        // Max distance which the robot can travel in one time step + scaling
        // TODO (#2370): This constant is calculated multiple times.
        float path_radius = (max_speed_ * simulator_->getTimeStep()) / 2;
        auto path_points  = {PathPoint(
            Vector(destination.x_meters(), destination.y_meters()), speed_at_dest)};
        path              = AgentPath(path_points, path_radius);

        // Update static obstacles
        std::set<TbotsProto::MotionConstraint> motion_constraints;
        for (int constraint_int : motion_control.motion_constraints())
        {
            if (!TbotsProto::MotionConstraint_IsValid(constraint_int))
            {
                continue;
            }

            const auto constraint =
                static_cast<TbotsProto::MotionConstraint>(constraint_int);
            auto new_obstacles =
                obstacle_factory.createFromMotionConstraint(constraint, world);
            if (constraint == TbotsProto::MotionConstraint::HALF_METER_AROUND_BALL)
            {
                ball_obstacle = new_obstacles[0];
            }
            else
            {
                static_obstacles.insert(static_obstacles.end(), new_obstacles.begin(),
                                        new_obstacles.end());
            }
        }
    }
    setPath(path);
}

void HRVOAgent::computeNeighbors(double neighbor_dist_threshold)
{
    // Re-calculate all agents (neighbors) within the distance threshold
    // which we want to create velocity obstacles for
    neighbors_.clear();
    simulator_->getKdTree()->query(this, neighbor_dist_threshold);
}

void HRVOAgent::computeVelocityObstacles()
{
    velocity_obstacles_.clear();
    velocity_obstacles_.reserve(neighbors_.size());

    const auto current_path_point_opt = getPath().getCurrentPathPoint();
    if (!current_path_point_opt.has_value())
    {
        // Don't draw any velocity obstacles if we do not have a destination
        return;
    }

    // Only consider agents within this distance away from our position
    auto current_destination = current_path_point_opt.value().getPosition();
    double dist_to_obstacle_threshold =
        std::min(static_cast<double>(max_neighbor_dist),
                 (getPosition() - current_destination).length());

    // Create Velocity Obstacles for neighboring agents
    computeNeighbors(dist_to_obstacle_threshold);
    for (const auto &neighbor : neighbors_)
    {
        std::shared_ptr<Agent> other_agent = simulator_->getAgents()[neighbor.second];
        VelocityObstacle velocity_obstacle = other_agent->createVelocityObstacle(*this);
        velocity_obstacles_.push_back(velocity_obstacle);
    }

    // Create Velocity Obstacles for nearby static obstacles
    Point agent_position_point(getPosition());
    Circle circle_rep_of_agent(agent_position_point, radius_);
    Segment path(agent_position_point, Point(current_destination));
    for (const auto &obstacle : static_obstacles)
    {
        double dist_agent_to_obstacle = obstacle->distance(agent_position_point);

        // Set of heuristics to minimize the amount of velocity obstacles
        if ((obstacle->intersects(path) ||
             dist_agent_to_obstacle < 2 * ROBOT_MAX_RADIUS_METERS) &&
            !obstacle->contains(agent_position_point))
        {
            VelocityObstacle velocity_obstacle =
                obstacle->generateVelocityObstacle(circle_rep_of_agent, Vector());
            velocity_obstacles_.push_back(velocity_obstacle);
        }
    }

    // The conditions for creating a velocity obstacle for the ball are different,
    // since the ball is a dynamic obstacle (not considered by the path planner)
    // and `generateVelocityObstacle` can create valid velocity obstacles for agents
    // contained in a circle.
    if (ball_obstacle.has_value())
    {
        auto obstacle = ball_obstacle.value();
        if (obstacle->intersects(path))
        {
            VelocityObstacle velocity_obstacle =
                obstacle->generateVelocityObstacle(circle_rep_of_agent, Vector());
            velocity_obstacles_.push_back(velocity_obstacle);
        }
    }
}

VelocityObstacle HRVOAgent::createVelocityObstacle(const Agent &other_agent)
{
    Circle obstacle_agent_circle(Point(getPosition()), radius_);
    Circle moving_agent_circle(Point(other_agent.getPosition()), radius_);
    auto vo = generateVelocityObstacle(obstacle_agent_circle, moving_agent_circle,
                                       getVelocity());

    // Convert velocity obstacle to hybrid reciprocal velocity obstacle (HRVO)
    // by shifting one side of the velocity obstacle to share the responsibility
    // of avoiding collision with other agent. This assumes that the other agent will also
    // be running HRVO
    // Refer to: https://gamma.cs.unc.edu/HRVO/HRVO-T-RO.pdf#page=2
    Vector vo_side;
    Vector rvo_side;
    if ((other_agent.getPrefVelocity() - pref_velocity_)
            .isClockwiseOf(position_ - other_agent.getPosition()))
    {
        vo_side  = vo.getLeftSide();
        rvo_side = vo.getRightSide();
    }
    else
    {
        // Vise versa of above
        vo_side  = vo.getRightSide();
        rvo_side = vo.getLeftSide();
    }
    Vector rvo_apex = (pref_velocity_ + other_agent.getPrefVelocity()) / 2;
    Line vo_side_line(Point(vo.getApex()), Point(vo.getApex() + vo_side));
    Line rvo_side_line(Point(rvo_apex), Point(rvo_apex + rvo_side));

    Vector hrvo_apex            = vo.getApex();
    auto intersection_point_opt = intersection(vo_side_line, rvo_side_line);
    if (intersection_point_opt.has_value())
    {
        hrvo_apex = intersection_point_opt.value().toVector();
    }

    return VelocityObstacle(hrvo_apex, vo.getLeftSide(), vo.getRightSide());
}

void HRVOAgent::computeNewVelocity()
{
    // Based on The Hybrid Reciprocal Velocity Obstacle paper:
    // https://gamma.cs.unc.edu/HRVO/HRVO-T-RO.pdf
    computePreferredVelocity();
    computeVelocityObstacles();

    // Find candidate velocities which this agent can take to avoid collision
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

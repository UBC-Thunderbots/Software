#include "software/ai/navigator/path_planner/hrvo/hrvo_agent.h"

HRVOAgent::HRVOAgent(RobotId robot_id, const RobotState &robot_state,
                     const RobotPath &path, double radius, double max_speed,
                     double max_accel, double max_radius_inflation)
    : Agent(robot_id, robot_state, path, radius, max_speed, max_accel,
            max_radius_inflation),
      obstacle_factory(TbotsProto::RobotNavigationObstacleConfig()),
      neighbours()
{
}

void HRVOAgent::updatePrimitive(const TbotsProto::Primitive &new_primitive,
                                const World &world, Duration time_step)
{
    RobotPath path;
    static_obstacles.clear();
    ball_obstacle = std::nullopt;
    if (new_primitive.has_move())
    {
        const auto &motion_control = new_primitive.move().motion_control();
        float speed_at_dest        = new_primitive.move().final_speed_m_per_s();
        float new_max_speed        = new_primitive.move().max_speed_m_per_s();
        this->max_speed            = new_max_speed;

        // TODO (#2418): Update implementation of Primitive to support
        // multiple path points and remove this check
        if (motion_control.path().points().size() < 2)
        {
            LOG(WARNING) << "Empty path: " << motion_control.path().points().size()
                         << std::endl;
            return;
        }

        auto destination = motion_control.path().points().at(1);

        Point destination_point = Point(destination.x_meters(), destination.y_meters());

        // Max distance which the robot can travel in one time step + scaling
        // TODO (#2370): This constant is calculated multiple    times.
        double path_radius = (max_speed * time_step.toSeconds()) / 2;
        auto path_points   = {PathPoint(destination_point, speed_at_dest)};
        path               = RobotPath(path_points, path_radius);

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

            // TODO (#2871): This assumes the constraint first obstacle is the ball,
            // which may not be the case;
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
    this->path = path;
}

std::vector<RobotId> HRVOAgent::computeNeighbors(
    const std::map<RobotId, std::shared_ptr<Agent>> &robots)
{
    // Only consider agents within this distance away from our position
    Point current_destination = path.getCurrentPathPoint()->getPosition();
    double dist_to_obstacle_threshold_squared =
        std::min(std::pow(MAX_NEIGHBOR_SEARCH_DIST, 2),
                 (position - current_destination).lengthSquared());

    auto compare = [&](const std::pair<RobotId, Point> &r1,
                       const std::pair<RobotId, Point> &r2) {
        return distanceSquared(r1.second, r2.second);
    };

    std::vector<std::pair<RobotId, Point>> robot_list;

    // transform map into list
    std::transform(robots.begin(), robots.end(), std::back_inserter(robot_list),
                   [&](const std::pair<RobotId, std::shared_ptr<Agent>> &id_robot_pair) {
                       return std::make_pair(id_robot_pair.first,
                                             id_robot_pair.second->getPosition());
                   });

    // remove this robot
    robot_list.erase(std::remove_if(robot_list.begin(), robot_list.end(),
                                    [this](const std::pair<RobotId, Point> &robot) {
                                        return robot_id == robot.first;
                                    }),
                     robot_list.end());

    // run brute force nn search
    std::vector<std::pair<RobotId, Point>> neighbors =
        findNeighboursInThreshold(std::make_pair(robot_id, position), robot_list,
                                  dist_to_obstacle_threshold_squared, compare);

    // unzip and return id from id-robot pairs
    std::vector<unsigned int> neighbor_ids;
    std::transform(neighbors.begin(), neighbors.end(), std::back_inserter(neighbor_ids),
                   [&](const std::pair<RobotId, Point> &id_neighbour_pair) {
                       return id_neighbour_pair.first;
                   });

    return neighbor_ids;
}

void HRVOAgent::computeVelocityObstacles(
    const std::map<RobotId, std::shared_ptr<Agent>> &robots)
{
    velocity_obstacles.reserve(robots.size());

    const auto current_path_point_opt = getPath().getCurrentPathPoint();
    auto current_destination          = current_path_point_opt.value().getPosition();

    if (!current_path_point_opt.has_value())
    {
        // Don't draw any velocity obstacles if we do not have a destination
        return;
    }

    // Create Velocity Obstacles for neighboring agents
    std::vector<unsigned int> neighbour_ids = computeNeighbors(robots);

    neighbours.clear();
    for (const auto &neighbor : neighbour_ids)
    {
        std::shared_ptr<Agent> other_agent = robots.find(neighbor)->second;
        neighbours.push_back(other_agent);
        VelocityObstacle velocity_obstacle = other_agent->createVelocityObstacle(*this);
        velocity_obstacles.push_back(velocity_obstacle);
    }

    // Create Velocity Obstacles for nearby static obstacles
    Point agent_position_point(position);
    Circle circle_rep_of_agent(agent_position_point, radius);
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
            velocity_obstacles.push_back(velocity_obstacle);
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
            velocity_obstacles.push_back(velocity_obstacle);
        }
    }
}

VelocityObstacle HRVOAgent::createVelocityObstacle(const Agent &other_agent)
{
    Circle moving_agent_circle(other_agent.getPosition(), radius);
    Circle obstacle_agent_circle(position, radius);
    auto vo =
        generateVelocityObstacle(obstacle_agent_circle, moving_agent_circle, velocity);

    // Convert velocity obstacle to hybrid reciprocal velocity obstacle (HRVO)
    // by shifting one side of the velocity obstacle to share the responsibility
    // of avoiding collision with other agent. This assumes that the other agent will also
    // be running HRVO
    // Refer to: https://gamma.cs.unc.edu/HRVO/HRVO-T-RO.pdf#page=2
    // Github: https://github.com/snape/HRVO
    Vector vo_side;
    Vector rvo_side;
    if ((other_agent.getPreferredVelocity() - preferred_velocity)
            .isClockwiseOf(position - other_agent.getPosition()))
    {
        vo_side  = vo.getLeftSide();
        rvo_side = vo.getRightSide();
    }
    else
    {
        // Vice versa of above
        vo_side  = vo.getRightSide();
        rvo_side = vo.getLeftSide();
    }
    Vector rvo_apex = (preferred_velocity + other_agent.getPreferredVelocity()) / 2;
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

void HRVOAgent::computeNewVelocity(
    const std::map<unsigned int, std::shared_ptr<Agent>> &agents, Duration time_step)
{
    // Based on The Hybrid Reciprocal Velocity Obstacle paper:
    // https://gamma.cs.unc.edu/HRVO/HRVO-T-RO.pdf

    computeVelocityObstacles(agents);

    const auto pref_velocity = computePreferredVelocity(time_step);

    // key is difference in length squared between PREFERRED and ACTUAL velocity
    std::multimap<double, CandidateVelocity> candidates;

    // verifies that the candidate speed is realistic and adds it to the possible
    // candidates
    auto addToCandidateListIfValid = [&](const CandidateVelocity &c) {
        if (c.velocity.lengthSquared() < max_speed * max_speed)
        {
            candidates.insert(
                std::make_pair((pref_velocity - c.velocity).lengthSquared(), c));
        }
    };

    // Find candidate velocities which this agent can take to avoid collision.
    // if small enough, add preferred velocity as candidate velocity or a normalized
    // version of it otherwise
    Vector candidate_velocity;

    if (pref_velocity.lengthSquared() < max_speed * max_speed)
    {
        candidate_velocity = pref_velocity;
    }
    else
    {
        candidate_velocity = pref_velocity.normalize(max_speed);
    }

    auto pref_candidate =
        CandidateVelocity(candidate_velocity, std::numeric_limits<int>::max(),
                          std::numeric_limits<int>::max());

    candidates.insert(std::make_pair(
        (pref_velocity - pref_candidate.velocity).lengthSquared(), pref_candidate));

    // this adds candidate points that are projections of the preferred velocity onto the
    // line segment of each obstacle
    for (int i = 0; i < static_cast<int>(velocity_obstacles.size()); ++i)
    {
        const Vector apex_to_pref_velocity =
            pref_velocity - velocity_obstacles[i].getApex();
        const double dot_product_1 =
            apex_to_pref_velocity.dot(velocity_obstacles[i].getRightSide());
        const double dot_product_2 =
            apex_to_pref_velocity.dot(velocity_obstacles[i].getLeftSide());

        if (dot_product_1 > 0.0f &&
            velocity_obstacles[i].getRightSide().isClockwiseOf(apex_to_pref_velocity))
        {
            candidate_velocity = velocity_obstacles[i].getApex() +
                                 dot_product_1 * velocity_obstacles[i].getRightSide();
            addToCandidateListIfValid(CandidateVelocity(candidate_velocity, i, i));
        }

        if (dot_product_2 > 0.0f &&
            velocity_obstacles[i].getLeftSide().isCounterClockwiseOf(
                apex_to_pref_velocity))
        {
            candidate_velocity = velocity_obstacles[i].getApex() +
                                 dot_product_2 * velocity_obstacles[i].getLeftSide();
            addToCandidateListIfValid(CandidateVelocity(candidate_velocity, i, i));
        }
    }

    for (int j = 0; j < static_cast<int>(velocity_obstacles.size()); ++j)
    {
        double discriminant =
            max_speed * max_speed -
            std::pow((velocity_obstacles[j].getApex())
                         .determinant(velocity_obstacles[j].getRightSide()),
                     2.f);

        if (discriminant > 0.0f)
        {
            const double t1 = -(velocity_obstacles[j].getApex().dot(
                                  velocity_obstacles[j].getRightSide())) +
                              std::sqrt(discriminant);
            const double t2 = -(velocity_obstacles[j].getApex().dot(
                                  velocity_obstacles[j].getRightSide())) -
                              std::sqrt(discriminant);

            if (t1 >= 0.0f)
            {
                candidate_velocity = velocity_obstacles[j].getApex() +
                                     t1 * velocity_obstacles[j].getRightSide();
                candidates.insert(std::make_pair(
                    (pref_velocity - candidate_velocity).lengthSquared(),
                    CandidateVelocity(candidate_velocity, std::numeric_limits<int>::max(),
                                      j)));
            }

            if (t2 >= 0.0f)
            {
                candidate_velocity = velocity_obstacles[j].getApex() +
                                     t2 * velocity_obstacles[j].getRightSide();
                candidates.insert(std::make_pair(
                    (pref_velocity - candidate_velocity).lengthSquared(),
                    CandidateVelocity(candidate_velocity, std::numeric_limits<int>::max(),
                                      j)));
            }
        }

        discriminant = max_speed * max_speed -
                       std::pow((velocity_obstacles[j].getApex())
                                    .determinant(velocity_obstacles[j].getLeftSide()),
                                2.f);

        if (discriminant > 0.0f)
        {
            const double t1 = -(velocity_obstacles[j].getApex().dot(
                                  velocity_obstacles[j].getLeftSide())) +
                              std::sqrt(discriminant);
            const double t2 = -(velocity_obstacles[j].getApex().dot(
                                  velocity_obstacles[j].getLeftSide())) -
                              std::sqrt(discriminant);

            if (t1 >= 0.0f)
            {
                candidate_velocity = velocity_obstacles[j].getApex() +
                                     t1 * velocity_obstacles[j].getLeftSide();
                candidates.insert(std::make_pair(
                    (pref_velocity - candidate_velocity).lengthSquared(),
                    CandidateVelocity(candidate_velocity, std::numeric_limits<int>::max(),
                                      j)));
            }

            if (t2 >= 0.0f)
            {
                candidate_velocity = velocity_obstacles[j].getApex() +
                                     t2 * velocity_obstacles[j].getLeftSide();
                candidates.insert(std::make_pair(
                    (pref_velocity - candidate_velocity).lengthSquared(),
                    CandidateVelocity(candidate_velocity, std::numeric_limits<int>::max(),
                                      j)));
            }
        }
    }

    // intersection points of all velocity obstacles with each other
    for (int i = 0; i < static_cast<int>(velocity_obstacles.size()) - 1; ++i)
    {
        for (int j = i + 1; j < static_cast<int>(velocity_obstacles.size()); ++j)
        {
            double d = (velocity_obstacles[i].getRightSide())
                           .determinant(velocity_obstacles[j].getRightSide());

            if (d != 0.0f)
            {
                const double s =
                    (velocity_obstacles[j].getApex() - velocity_obstacles[i].getApex())
                        .determinant(velocity_obstacles[j].getRightSide()) /
                    d;
                const double t =
                    (velocity_obstacles[j].getApex() - velocity_obstacles[i].getApex())
                        .determinant(velocity_obstacles[i].getRightSide()) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate_velocity = velocity_obstacles[i].getApex() +
                                         s * velocity_obstacles[i].getRightSide();
                    addToCandidateListIfValid(
                        CandidateVelocity(candidate_velocity, i, j));
                }
            }

            d = (velocity_obstacles[i].getLeftSide())
                    .determinant(velocity_obstacles[j].getRightSide());

            if (d != 0.0f)
            {
                const double s =
                    (velocity_obstacles[j].getApex() - velocity_obstacles[i].getApex())
                        .determinant(velocity_obstacles[j].getRightSide()) /
                    d;
                const double t =
                    (velocity_obstacles[j].getApex() - velocity_obstacles[i].getApex())
                        .determinant(velocity_obstacles[i].getLeftSide()) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate_velocity = velocity_obstacles[i].getApex() +
                                         s * velocity_obstacles[i].getLeftSide();
                    addToCandidateListIfValid(
                        CandidateVelocity(candidate_velocity, i, j));
                }
            }

            d = (velocity_obstacles[i].getRightSide())
                    .determinant(velocity_obstacles[j].getLeftSide());

            if (d != 0.0f)
            {
                const double s =
                    (velocity_obstacles[j].getApex() - velocity_obstacles[i].getApex())
                        .determinant(velocity_obstacles[j].getLeftSide()) /
                    d;
                const double t =
                    (velocity_obstacles[j].getApex() - velocity_obstacles[i].getApex())
                        .determinant(velocity_obstacles[i].getRightSide()) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate_velocity = velocity_obstacles[i].getApex() +
                                         s * velocity_obstacles[i].getRightSide();
                    addToCandidateListIfValid(
                        CandidateVelocity(candidate_velocity, i, j));
                }
            }

            d = (velocity_obstacles[i].getLeftSide())
                    .determinant(velocity_obstacles[j].getLeftSide());

            if (d != 0.0f)
            {
                const double s =
                    (velocity_obstacles[j].getApex() - velocity_obstacles[i].getApex())
                        .determinant(velocity_obstacles[j].getLeftSide()) /
                    d;
                const double t =
                    (velocity_obstacles[j].getApex() - velocity_obstacles[i].getApex())
                        .determinant(velocity_obstacles[i].getLeftSide()) /
                    d;

                if (s >= 0.0f && t >= 0.0f)
                {
                    candidate_velocity = velocity_obstacles[i].getApex() +
                                         s * velocity_obstacles[i].getLeftSide();
                    addToCandidateListIfValid(
                        CandidateVelocity(candidate_velocity, i, j));
                }
            }
        }
    }
    // corresponds to the velocity obstacle that is furthest away when picking a velocity
    // we might collide with
    int optimal_furthest_away_obstacle = -1;

    new_velocity = Vector();

    // Choosing a candidate velocity has these goals:
    // - Pick a velocity as close as possible to the preferred velocity as long as it
    // doesn't lie in any velocity obstacle
    // - Failing the first condition, then choose the best candidate velocity as the new
    // velocity that minimizes collisions with the closest velocity obstacles
    // - Candidate multimap is organized by distance from preferred velocity so we pick
    // the first valid velocity
    for (const auto &dist_to_pref_velocity_sq_candidate_pair : candidates)
    {
        const auto candidate = dist_to_pref_velocity_sq_candidate_pair.second;
        std::optional<int> first_intersecting_velocity_obstacle =
            findIntersectingVelocityObstacle(candidate);

        // return as soon as we find a candidate velocity that doesn't intersect anything
        // and is fast
        if (isIdealCandidate(candidate))
        {
            new_velocity = candidate.velocity;
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
            new_velocity = candidate.velocity;
        }
    }
}

bool HRVOAgent::isIdealCandidate(const CandidateVelocity &candidate) const
{
    return !findIntersectingVelocityObstacle(candidate).has_value() &&
           isCandidateFast(candidate);
}

bool HRVOAgent::isCandidateSlow(const CandidateVelocity &candidate) const
{
    double min_pref_speed =
        std::abs(preferred_velocity.length()) * MIN_PREF_SPEED_MULTIPLIER;

    return std::abs(candidate.velocity.length()) < min_pref_speed;
}

bool HRVOAgent::isCandidateFast(const CandidateVelocity &candidate) const
{
    return !isCandidateSlow(candidate);
}

bool HRVOAgent::isCandidateFasterThanCurrentSpeed(
    const CandidateVelocity &candidate) const
{
    return std::abs(candidate.velocity.length()) > std::abs(new_velocity.length());
}

std::optional<int> HRVOAgent::findIntersectingVelocityObstacle(
    const CandidateVelocity &candidate) const
{
    for (int j = 0; j < static_cast<int>(velocity_obstacles.size()); ++j)
    {
        if (j != candidate.obstacle_indexes.first &&
            j != candidate.obstacle_indexes.second &&
            velocity_obstacles[j].containsVelocity(candidate.velocity))
        {
            return std::make_optional<int>(j);
        }
    }

    return std::nullopt;
}

Vector HRVOAgent::computePreferredVelocity(Duration time_step)
{
    double pref_speed   = max_speed * PREF_SPEED_SCALE;
    auto path_point_opt = path.getCurrentPathPoint();

    if (pref_speed <= 0.01f || max_accel <= 0.01f || path_point_opt == std::nullopt)
    {
        // Used to avoid edge cases with division by zero
        return Vector(0.f, 0.f);
    }

    Point goal_position  = path_point_opt.value().getPosition();
    double speed_at_goal = path_point_opt.value().getSpeed();

    Vector dist_vector_to_goal = goal_position - position;
    auto dist_to_goal          = static_cast<float>(dist_vector_to_goal.length());

    // d = (Vf^2 - Vi^2) / 2a
    double start_linear_deceleration_distance =
        std::abs((std::pow(speed_at_goal, 2) - std::pow(pref_speed, 2)) /
                 (2 * max_accel)) *
        DECEL_DIST_MULTIPLIER;

    if (dist_to_goal < start_linear_deceleration_distance)
    {
        // velocity given linear deceleration, distance away from goal, and desired final
        // speed
        // v_pref = sqrt(v_goal^2 + 2 * a * d_remainingToDestination)
        double curr_pref_speed =
            static_cast<double>(
                std::sqrt(std::pow(speed_at_goal, 2) + 2 * max_accel * dist_to_goal)) *
            DECEL_PREF_SPEED_MULTIPLIER;
        Vector ideal_pref_velocity = dist_vector_to_goal.normalize(curr_pref_speed);

        // Limit the preferred velocity to the kinematic limits
        const Vector dv = ideal_pref_velocity - velocity;
        if (dv.length() <= max_accel * time_step.toSeconds())
        {
            return ideal_pref_velocity;
        }
        else
        {
            // Calculate the maximum velocity towards the preferred velocity, given the
            // acceleration constraint
            return velocity + dv.normalize(max_accel * time_step.toSeconds());
        }
    }
    else
    {
        // Accelerate to preferred speed
        // v_pref = v_now + a * t
        double curr_pref_speed =
            std::min(static_cast<double>(pref_speed),
                     velocity.length() + max_accel * time_step.toSeconds());
        return dist_vector_to_goal.normalize(curr_pref_speed);
    }
}

std::optional<ObstaclePtr> HRVOAgent::getBallObstacle()
{
    return ball_obstacle;
}

std::vector<VelocityObstacle> HRVOAgent::getVelocityObstacles()
{
    return velocity_obstacles;
}

void HRVOAgent::visualize(TeamColour friendly_team_colour)
{
    TbotsProto::HRVOVisualization hrvo_visualization;
    // Visualize all neighbours
    for (const auto &robot : neighbours)
    {
        Point position(robot->getPosition());
        *(hrvo_visualization.add_robots()) =
            *createCircleProto(Circle(position, robot->radius));
    }

    std::vector<TbotsProto::VelocityObstacle> vo_protos;
    for (const VelocityObstacle &vo : getVelocityObstacles())
    {
        vo_protos.emplace_back(
            *createVelocityObstacleProto(vo, getPosition().toVector()));
    }
    hrvo_visualization.set_robot_id(robot_id);

    *(hrvo_visualization.mutable_velocity_obstacles()) = {vo_protos.begin(),
                                                          vo_protos.end()};

    // Visualize the ball obstacle
    if (getBallObstacle().has_value())
    {
        TbotsProto::Circle ball_circle =
            getBallObstacle().value()->createObstacleProto().circle()[0];
        *(hrvo_visualization.add_robots()) = ball_circle;
    }

    if (friendly_team_colour == TeamColour::YELLOW)
    {
        LOG(VISUALIZE, YELLOW_HRVO_PATH) << hrvo_visualization;
    }
    else
    {
        LOG(VISUALIZE, BLUE_HRVO_PATH) << hrvo_visualization;
    }
}

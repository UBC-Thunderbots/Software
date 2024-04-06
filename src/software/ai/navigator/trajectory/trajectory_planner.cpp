#include "software/ai/navigator/trajectory/trajectory_planner.h"

#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"

TrajectoryPlanner::TrajectoryPlanner()
    : relative_sub_destinations(getRelativeSubDestinations())
{
}

std::vector<Vector> TrajectoryPlanner::getRelativeSubDestinations()
{
    // A set of sub destinations positioned around a circle relative to the robot.
    std::vector<Vector> relative_sub_destinations;
    const Angle sub_angles = Angle::full() / NUM_SUB_DESTINATION_ANGLES;
    for (const double distance : SUB_DESTINATION_DISTANCES_METERS)
    {
        for (unsigned int i = 0; i < NUM_SUB_DESTINATION_ANGLES; ++i)
        {
            Angle angle = sub_angles * i;
            relative_sub_destinations.emplace_back(
                Vector::createFromAngle(angle).normalize(distance));
        }
    }
    return relative_sub_destinations;
}

std::vector<Point> TrajectoryPlanner::getSubDestinations(
    const Point &start, const Point &destination, const Rectangle &navigable_area) const
{
    // Convert the relative sub destinations to actual sub destination points
    // and filter out undesirable sub destinations to reduce trajectory sampling.
    std::vector<Point> sub_destinations;
    Angle direction = (destination - start).orientation();
    sub_destinations.reserve(relative_sub_destinations.size());

    for (const Vector &relative_sub_dest : relative_sub_destinations)
    {
        Angle sub_dest_angle_to_dest = relative_sub_dest.orientation().minDiff(direction);
        if (sub_dest_angle_to_dest < MIN_SUB_DESTINATION_ANGLE ||
            sub_dest_angle_to_dest > MAX_SUB_DESTINATION_ANGLE)
        {
            continue;
        }

        Point sub_dest = start + relative_sub_dest;
        if (!contains(navigable_area, sub_dest))
        {
            continue;
        }
        sub_destinations.emplace_back(sub_dest);
    }
    return sub_destinations;
}

std::optional<TrajectoryPath> TrajectoryPlanner::findTrajectory(
    const Point &start, const Point &destination, const Vector &initial_velocity,
    const KinematicConstraints &constraints, const std::vector<ObstaclePtr> &obstacles,
    const Rectangle &navigable_area, const std::optional<Point> &prev_sub_destination)
{
    if (constraints.getMaxVelocity() <= 0.0 || constraints.getMaxAcceleration() <= 0.0 ||
        constraints.getMaxDeceleration() <= 0.0)
    {
        return std::nullopt;
    }

    TrajectoryPathWithCost best_traj_with_cost = getDirectTrajectoryWithCost(
        start, destination, initial_velocity, constraints, obstacles);

    // Return direct trajectory to the destination if it doesn't have any collisions
    if (!best_traj_with_cost.collides())
    {
        return best_traj_with_cost.traj_path;
    }

    // Sample trajectory paths by trying different sub destinations and connection times
    // and store the best trajectory path (min cost)
    for (const Point &sub_dest : getSubDestinations(start, destination, navigable_area))
    {
        // Generate a direct trajectory to the sub destination
        TrajectoryPathWithCost sub_trajectory = getDirectTrajectoryWithCost(
            start, sub_dest, initial_velocity, constraints, obstacles);

        // Prefer sub destinations that are closer to the previous sub destination.
        // This is used to avoid oscillation between two sub destinations that return a
        // trajectory with the similar cost.
        double cost_offset = 0.0;
        if (prev_sub_destination.has_value() &&
            distance(sub_dest, prev_sub_destination.value()) <
                SUB_DESTINATION_CLOSE_BONUS_THRESHOLD_METERS)
        {
            cost_offset = SUB_DESTINATION_CLOSE_BONUS_COST;
        }

        for (double connection_time = SUB_DESTINATION_STEP_INTERVAL_SEC;
             connection_time <= sub_trajectory.traj_path.getTotalTime();
             connection_time += SUB_DESTINATION_STEP_INTERVAL_SEC)
        {
            // Branch off of a copy of the initial trajectory at connection_time
            // to move towards the actual destination.
            TrajectoryPath traj_path_to_dest = sub_trajectory.traj_path;
            traj_path_to_dest.append(connection_time, destination, constraints);

            // Return early for this sub destination if the trajectory can
            // not have a lower cost than the best trajectory.
            if (traj_path_to_dest.getTotalTime() >= best_traj_with_cost.cost)
            {
                break;
            }

            TrajectoryPathWithCost full_traj_with_cost = getTrajectoryWithCost(
                traj_path_to_dest, obstacles, sub_trajectory, connection_time);
            full_traj_with_cost.cost += cost_offset;
            if (full_traj_with_cost.cost < best_traj_with_cost.cost)
            {
                best_traj_with_cost = full_traj_with_cost;
            }

            // Later connection_times will generally result in a longer trajectory
            // duration, thus, if this trajectory does not have a collision, then we can
            // not get a better trajectory with a later connection_time Alternatively, if
            // this trajectory does collide and their exists a trajectory that doesn't
            // collide and has a shorter duration, then we're not going to find a better
            // trajectory by increasing the connection time.
            if (!full_traj_with_cost.collides() ||
                (!best_traj_with_cost.collides() &&
                 full_traj_with_cost.traj_path.getTotalTime() >
                     best_traj_with_cost.traj_path.getTotalTime()))
            {
                break;
            }
        }
    }

    return best_traj_with_cost.traj_path;
}

TrajectoryPathWithCost TrajectoryPlanner::getDirectTrajectoryWithCost(
    const Point &start, const Point &destination, const Vector &initial_velocity,
    const KinematicConstraints &constraints, const std::vector<ObstaclePtr> &obstacles)
{
    return getTrajectoryWithCost(
        TrajectoryPath(std::make_shared<BangBangTrajectory2D>(
                           start, destination, initial_velocity, constraints),
                       BangBangTrajectory2D::generator),
        obstacles, std::nullopt, std::nullopt);
}

TrajectoryPathWithCost TrajectoryPlanner::getTrajectoryWithCost(
    const TrajectoryPath &trajectory, const std::vector<ObstaclePtr> &obstacles,
    const std::optional<TrajectoryPathWithCost> &sub_traj_with_cost,
    const std::optional<double> sub_traj_duration_s)
{
    TrajectoryPathWithCost traj_with_cost(trajectory);

    const double search_end_time_s =
        std::min(trajectory.getTotalTime(), MAX_FUTURE_COLLISION_CHECK_SEC);

    /**
     * Find the start duration before the trajectory leaves all obstacles
     */
    double first_non_collision_time;
    // Avoid finding the first non-collision time if the cache sub-trajectory
    // has a collision time.
    if (sub_traj_with_cost.has_value() &&
        sub_traj_with_cost->collision_duration_front_s < sub_traj_duration_s)
    {
        first_non_collision_time = sub_traj_with_cost->collision_duration_front_s;
    }
    else
    {
        first_non_collision_time =
            getFirstNonCollisionTime(trajectory, obstacles, search_end_time_s);
    }
    traj_with_cost.collision_duration_front_s = first_non_collision_time;

    /**
     * Find the duration we're within an obstacle before search_end_time_s
     */
    double last_non_collision_time =
        getLastNonCollisionTime(trajectory, obstacles, search_end_time_s);
    traj_with_cost.collision_duration_back_s =
        search_end_time_s - last_non_collision_time;

    /**
     * Get the first collision time, excluding the time at the start and end of path
     * that we may be in an obstacle for.
     */
    if (sub_traj_with_cost.has_value() &&
        sub_traj_with_cost->first_collision_time_s < sub_traj_duration_s)
    {
        traj_with_cost.first_collision_time_s =
            sub_traj_with_cost->first_collision_time_s;
        traj_with_cost.colliding_obstacle = sub_traj_with_cost->colliding_obstacle;
    }
    else
    {
        std::pair<double, ObstaclePtr> collision = getFirstCollisionTime(
            trajectory, obstacles, first_non_collision_time, last_non_collision_time);
        traj_with_cost.first_collision_time_s = collision.first;
        traj_with_cost.colliding_obstacle     = collision.second;
    }

    traj_with_cost.cost = calculateCost(traj_with_cost);

    return traj_with_cost;
}

double TrajectoryPlanner::calculateCost(
    const TrajectoryPathWithCost &traj_with_cost) const
{
    double total_cost = traj_with_cost.traj_path.getTotalTime();

    // Add a large cost if the trajectory collides with an obstacle
    // Note that this ignores collisions that may be in at the
    // start of the trajectory as those are unavoidable by all trajectories.
    if (traj_with_cost.colliding_obstacle != nullptr)
    {
        total_cost += 6.0;
    }

    // The closer the collision is to the destination, the lower its cost will be
    Point first_collision_position =
        traj_with_cost.traj_path.getPosition(traj_with_cost.first_collision_time_s);
    Point destination = traj_with_cost.traj_path.getDestination();
    total_cost += (first_collision_position - destination).length();

    total_cost += std::max(
        0.0, (MAX_FUTURE_COLLISION_CHECK_SEC - traj_with_cost.first_collision_time_s));

    total_cost += 3 * traj_with_cost.collision_duration_front_s;

    total_cost += 1 * traj_with_cost.collision_duration_back_s;

    return total_cost;
}

double TrajectoryPlanner::getFirstNonCollisionTime(
    const TrajectoryPath &traj_path, const std::vector<ObstaclePtr> &obstacles,
    const double search_end_time_s) const
{
    double path_duration = traj_path.getTotalTime();
    for (double time = 0.0; time <= search_end_time_s;
         time += FORWARD_COLLISION_CHECK_STEP_INTERVAL_SEC)
    {
        Point position       = traj_path.getPosition(time);
        bool collision_found = false;
        for (const ObstaclePtr &obstacle : obstacles)
        {
            if (obstacle->contains(position, time))
            {
                collision_found = true;
                break;
            }
        }

        if (!collision_found)
        {
            return time;
        }
    }
    return path_duration;
}

std::pair<double, ObstaclePtr> TrajectoryPlanner::getFirstCollisionTime(
    const TrajectoryPath &traj_path, const std::vector<ObstaclePtr> &obstacles,
    const double start_time_s, const double search_end_time_s) const
{
    for (double time = start_time_s; time <= search_end_time_s;
         time += COLLISION_CHECK_STEP_INTERVAL_SEC)
    {
        Point position = traj_path.getPosition(time);
        for (const ObstaclePtr &obstacle : obstacles)
        {
            if (obstacle->contains(position, time))
            {
                return std::make_pair(time, obstacle);
            }
        }
    }

    // No collision found
    return std::make_pair(std::numeric_limits<double>::max(), nullptr);
}

double TrajectoryPlanner::getLastNonCollisionTime(
    const TrajectoryPath &traj_path, const std::vector<ObstaclePtr> &obstacles,
    const double search_end_time_s) const
{
    for (double time = search_end_time_s; time >= 0.0;
         time -= COLLISION_CHECK_STEP_INTERVAL_SEC)
    {
        Point position       = traj_path.getPosition(time);
        bool collision_found = false;
        for (const ObstaclePtr &obstacle : obstacles)
        {
            if (obstacle->contains(position, time))
            {
                collision_found = true;
                break;
            }
        }

        if (!collision_found)
        {
            return time;
        }
    }
    return search_end_time_s;
}

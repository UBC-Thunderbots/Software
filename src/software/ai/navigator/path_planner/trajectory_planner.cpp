#include "software/ai/navigator/path_planner/trajectory_planner.h"

#include "external/tracy/public/tracy/Tracy.hpp"

TrajectoryPlanner::TrajectoryPlanner()
{
    // Initialize the relative sub-destinations array
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
}

TrajectoryPath TrajectoryPlanner::findTrajectory(
    const Point &start, const Point &destination, const Vector &initial_velocity,
    const KinematicConstraints &constraints, const std::vector<ObstaclePtr> &obstacles,
    const Rectangle &navigable_area)
{
    ZoneScopedN("findTrajectory");
    static long int total_time = 0;
    static int num_calls       = 0;
    auto start_time            = std::chrono::high_resolution_clock::now();

    // TODO: This can probably be shared between all findTrajectory calls in the same
    // tick.
    //       Should create a wrapper which automatically creates this (and hides it) and
    //       stores the obstacles
    aabb::Tree tree(2, 0.0, {false, false},
                    {navigable_area.xLength(), navigable_area.yLength()},
                    static_cast<unsigned int>(obstacles.size()), false);
    {
        ZoneScopedN("fillObstacleTree");
        for (unsigned int i = 0; i < obstacles.size(); i++)
        {
            Rectangle aabb         = obstacles[i]->axisAlignedBoundingBox();
            std::vector aabb_lower = {aabb.negXNegYCorner().x(), aabb.negXNegYCorner().y()};
            std::vector aabb_upper = {aabb.posXPosYCorner().x(), aabb.posXPosYCorner().y()};
            tree.insertParticle(i, aabb_lower, aabb_upper);
        }
    }

    TrajectoryPathWithCost best_traj_with_cost = getDirectTrajectoryWithCost(start, destination, initial_velocity, constraints, tree, obstacles);

    // Return direct trajectory to the destination if it doesn't have any collisions
    if (!best_traj_with_cost.collides())
    {
        return best_traj_with_cost.traj_path;
    }
    // std::cout << "Direct trajectory collides" << std::endl;

    std::vector<Point> sub_destinations;
    sub_destinations.reserve(relative_sub_destinations.size());
    for (const Vector &relative_sub_dest : relative_sub_destinations)
    {
        Point sub_dest = start + relative_sub_dest;
        if (contains(navigable_area, sub_dest))
        {
            sub_destinations.emplace_back(sub_dest);
        }
    }

    {
        ZoneScopedN("generateTrajectories");

        // Add trajectories that go through sub-destinations
        for (const Point &sub_dest : sub_destinations)
        {
            TrajectoryPathWithCost sub_trajectory = getDirectTrajectoryWithCost(start, sub_dest, initial_velocity, constraints, tree, obstacles);

            for (double connection_time = SUB_DESTINATION_STEP_INTERVAL_SEC;
                 connection_time <= sub_trajectory.traj_path.getTotalTime();
                 connection_time += SUB_DESTINATION_STEP_INTERVAL_SEC)
            {
                ZoneScopedN("pushBackAndAppendTrajectoy");
                // Copy the sub trajectory, then append a trajectory to the
                // actual destination at connection_time
                TrajectoryPath traj_path_to_dest = sub_trajectory.traj_path;
                traj_path_to_dest.append(constraints, connection_time, destination);
                TrajectoryPathWithCost full_traj_with_cost = getTrajectoryWithCost(traj_path_to_dest, tree, obstacles, sub_trajectory, connection_time);
                // TODO: If full_traj_with_cost doesn't have any collisions, should we continue to next iter?
                //       i.e. is it possible that with a later connection_time we get an improved score?!
                if (full_traj_with_cost.cost < best_traj_with_cost.cost)
                {
                    best_traj_with_cost = full_traj_with_cost;
                }
            }
        }
    }

    // TODO: Added for debugging
    auto end_time = std::chrono::high_resolution_clock::now();
    total_time +=
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time)
            .count();

    num_calls++;
    if (num_calls % 1000 == 0)
    {
         std::cout << "Average total time: " << total_time / num_calls << "us" << std::endl;
        total_time = 0;
        num_calls  = 0;
    }

    return best_traj_with_cost.traj_path;
}

double TrajectoryPlanner::calculateCost(const TrajectoryPath &trajectory_path,
                                        aabb::Tree &obstacle_tree,
                                        const std::vector<ObstaclePtr> &obstacles)
{
    double cost = trajectory_path.getTotalTime();

    // TODO: Only get AABB up to MAX_FUTURE_COLLISION_CHECK into the future
    std::set<unsigned int> possible_collisions_indices;
    for (const Rectangle &bounding_box : trajectory_path.getBoundingBoxes())
    {
        std::vector<unsigned int> bb_collisions = obstacle_tree.query(aabb::AABB(
            {bounding_box.negXNegYCorner().x(), bounding_box.negXNegYCorner().y()},
            {bounding_box.posXPosYCorner().x(), bounding_box.posXPosYCorner().y()}));
        possible_collisions_indices.insert(bb_collisions.begin(), bb_collisions.end());
    }

    if (possible_collisions_indices.empty())
    {
        return cost;
    }

    bool starts_in_collision = false;
    bool first_iteration     = true;
    bool collision_found     = false;
    double earliest_collision_time =
        trajectory_path.getTotalTime() + 10.0;

    for (double time = 0.0; time < MAX_FUTURE_COLLISION_CHECK_SEC; time += 0.1)
    {
        Point position = trajectory_path.getPosition(time);
        for (unsigned int obstacle_index : possible_collisions_indices)
        {
            // TODO: Consider updating the contains implementation (for polygon
            // specifically if it's used)
            //       to check AABB contains first...
            // Do actual more expensive collision check
            if (obstacles[obstacle_index]->contains(position))
            {
                collision_found = true;
                if (time < earliest_collision_time)
                {
                    earliest_collision_time = time;
                }

                if (first_iteration)
                {
                    starts_in_collision = true;
                }
            }

            first_iteration = false;
            // TODO: Add cost depending on length of collision?!
        }
    }

    if (collision_found)
    {
        cost += PATH_WITH_COLLISION_COST;
        cost += (trajectory_path.getTotalTime() - earliest_collision_time);
    }

    if (starts_in_collision)
    {
        // TODO;
        cost += 0.0;
    }

    return cost;
}

TrajectoryPathWithCost
TrajectoryPlanner::getDirectTrajectoryWithCost(const Point &start, const Point &destination, const Vector &initial_velocity,
                                               const KinematicConstraints &constraints, aabb::Tree &obstacle_tree,
                                               const std::vector<ObstaclePtr> &obstacles)
{
    // TODO: Consider removing this function and just using getTrajectoryWithCost
    return getTrajectoryWithCost(TrajectoryPath(BangBangTrajectory2D(
            start, destination, initial_velocity, constraints.getMaxVelocity(),
            constraints.getMaxAcceleration(), constraints.getMaxDeceleration())), obstacle_tree,
                                 obstacles, std::nullopt, std::nullopt);
}

TrajectoryPathWithCost
TrajectoryPlanner::getTrajectoryWithCost(const TrajectoryPath &trajectory, aabb::Tree &obstacle_tree,
                                         const std::vector<ObstaclePtr> &obstacles,
                                         const std::optional<TrajectoryPathWithCost>& sub_traj_with_cost,
                                         const std::optional<double> sub_traj_duration_sec)
{
    TrajectoryPathWithCost traj_with_cost(trajectory);

    std::set<unsigned int> possible_collisions_indices;
    for (const Rectangle &bounding_box : trajectory.getBoundingBoxes())
    {
        std::vector<unsigned int> bb_collisions = obstacle_tree.query(aabb::AABB(
                {bounding_box.negXNegYCorner().x(), bounding_box.negXNegYCorner().y()},
                {bounding_box.posXPosYCorner().x(), bounding_box.posXPosYCorner().y()}));
        possible_collisions_indices.insert(bb_collisions.begin(), bb_collisions.end());
    }

    double first_non_collision_time;
    if (sub_traj_with_cost.has_value() && sub_traj_with_cost->collision_duration_front < sub_traj_duration_sec)
    {
        first_non_collision_time = sub_traj_with_cost->collision_duration_front;
    }
    else
    {
        first_non_collision_time = getFirstNonCollisionTime(trajectory, possible_collisions_indices, obstacles);
    }
    traj_with_cost.collision_duration_front = first_non_collision_time;

    double last_non_collision_time = getLastNonCollisionTime(trajectory, possible_collisions_indices, obstacles);
    traj_with_cost.collision_duration_back = std::max(0.0, trajectory.getTotalTime() - last_non_collision_time);

    // Get the first collision time, excluding the time at the start and end of path
    // that we may be in an obstacle for.
    if (sub_traj_with_cost.has_value() && sub_traj_with_cost->first_collision_time < sub_traj_duration_sec)
    {
        traj_with_cost.first_collision_time = sub_traj_with_cost->first_collision_time;
        traj_with_cost.colliding_obstacle = sub_traj_with_cost->colliding_obstacle;
    }
    else
    {
        std::pair<double, ObstaclePtr> collision = getFirstCollisionTime(trajectory, possible_collisions_indices,
                                                                           obstacles, first_non_collision_time,
                                                                           last_non_collision_time);
        traj_with_cost.first_collision_time = collision.first;
        traj_with_cost.colliding_obstacle = collision.second;
    }

    // TODO: Add collision_penalty?! Tiger's does it for defense area based on how much it goes into it

    traj_with_cost.cost = calculateCost(traj_with_cost);

    return traj_with_cost;
}

double TrajectoryPlanner::calculateCost(const TrajectoryPathWithCost &traj_with_cost)
{
    double total_cost = traj_with_cost.traj_path.getTotalTime();

    // TODO: Tiger's does this dynamically?! collisionPenalty. Mainly for penalty area
    if (traj_with_cost.colliding_obstacle != nullptr)
    {
        total_cost += 3.0;
        // std::cout << "Colliding with obstacle: " << traj_with_cost.colliding_obstacle->toString() << " ";
    }

    Point first_collision_position = traj_with_cost.traj_path.getPosition(traj_with_cost.first_collision_time);
    Point destination = traj_with_cost.traj_path.getDestination();
    total_cost += (first_collision_position - destination).length();
    // std::cout << " + (first_collision_position - destination).length(): " << (first_collision_position - destination).length() << " ";

    total_cost += std::max(0.0, (MAX_FUTURE_COLLISION_CHECK_SEC - traj_with_cost.first_collision_time));
    // std::cout << " + std::max(0.0, (MAX_FUTURE_COLLISION_CHECK - traj_with_cost.first_collision_time)): " << std::max(0.0, (MAX_FUTURE_COLLISION_CHECK - traj_with_cost.first_collision_time)) << " ";

    total_cost += 3 * traj_with_cost.collision_duration_front;
    // std::cout << " + 3 * traj_with_cost.collision_duration_front: " << 3 * traj_with_cost.collision_duration_front << " ";

    total_cost += 1 * traj_with_cost.collision_duration_back;
    // std::cout << " + 1 * traj_with_cost.collision_duration_back: " << 1 * traj_with_cost.collision_duration_back << " = " << total_cost << std::endl;

    return total_cost;
}

double TrajectoryPlanner::getFirstNonCollisionTime(const TrajectoryPath &traj_path,
                                                     const std::set<unsigned int>& obstacle_indices,
                                                     const std::vector<ObstaclePtr> &obstacles) const
{
    double path_length = traj_path.getTotalTime();
    for (double time = 0.0;
         time <= std::min(path_length, MAX_FUTURE_COLLISION_CHECK_SEC);
         time += FORWARD_COLLISION_CHECK_STEP_INTERVAL_SEC)
    {
        Point position = traj_path.getPosition(time);
        bool collision_found = false;
        for (unsigned int obstacle_index : obstacle_indices)
        {
            if (obstacles[obstacle_index]->contains(position))
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
    return path_length;
}

std::pair<double, ObstaclePtr>
TrajectoryPlanner::getFirstCollisionTime(const TrajectoryPath &traj_path, const std::set<unsigned int> &obstacle_indices,
                                         const std::vector<ObstaclePtr> &obstacles, const double start_time_sec,
                                         const double stop_time_sec) const
{
    for (double time = start_time_sec;
         time <= std::min(stop_time_sec, MAX_FUTURE_COLLISION_CHECK_SEC);
         time += COLLISION_CHECK_STEP_INTERVAL_SEC)
    {
        Point position = traj_path.getPosition(time);
        for (unsigned int obstacle_index : obstacle_indices)
        {
            if (obstacles[obstacle_index]->contains(position))
            {
                return std::make_pair(time, obstacles[obstacle_index]);
            }
        }
    }
    // return double limit
    return std::make_pair(std::numeric_limits<double>::max(), nullptr);
}

double TrajectoryPlanner::getLastNonCollisionTime(const TrajectoryPath &traj_path, const std::set<unsigned int> &obstacle_indices,
                                                    const std::vector<ObstaclePtr> &obstacles) const
{
    double path_length = traj_path.getTotalTime();
    for (double time = std::min(path_length, MAX_FUTURE_COLLISION_CHECK_SEC);
         time >= 0.0;
         time -= COLLISION_CHECK_STEP_INTERVAL_SEC)
    {
        Point position = traj_path.getPosition(time);
        bool collision_found = false;
        for (unsigned int obstacle_index : obstacle_indices)
        {
            // TODO: Tiger's adds extra margin based on velocity
            if (obstacles[obstacle_index]->contains(position))
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
    return 0.0;
}

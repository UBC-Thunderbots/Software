#include "software/ai/navigator/path_planner/trajectory_planner.h"

#include "external/tracy/public/tracy/Tracy.hpp"
#include "proto/message_translation/tbots_protobuf.h"

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
    static long int total_time = 0;
    static int num_calls       = 0;
    auto start_time            = std::chrono::high_resolution_clock::now();

    // TODO: This can probably be shared between all findTrajectory calls in the same
    // tick.
    //       Should create a wrapper which automatically creates this (and hides it) and
    //       stores the obstacles
    aabb::Tree tree(2, 0.0, {false, false},
                    {navigable_area.xLength(), navigable_area.yLength()},
                    std::max(static_cast<unsigned int>(obstacles.size()), 1u), false);
    for (unsigned int i = 0; i < obstacles.size(); i++)
    {
        Rectangle aabb         = obstacles[i]->axisAlignedBoundingBox();
        std::vector aabb_lower = {aabb.negXNegYCorner().x(),
                                  aabb.negXNegYCorner().y()};
        std::vector aabb_upper = {aabb.posXPosYCorner().x(),
                                  aabb.posXPosYCorner().y()};
        tree.insertParticle(i, aabb_lower, aabb_upper);
    }

    TrajectoryPathWithCost best_traj_with_cost = getDirectTrajectoryWithCost(
        start, destination, initial_velocity, constraints, tree, obstacles);

    // Return direct trajectory to the destination if it doesn't have any collisions
    if (!best_traj_with_cost.collides())
    {
        last_sub_dest = std::nullopt;
        return best_traj_with_cost.traj_path;
    }

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

//    // TODO: Dont know last sub destination doesnt improve
//    if (last_sub_dest.has_value())
//    {
//        Point sub_dest = last_sub_dest.value();
//        TrajectoryPathWithCost sub_trajectory = getDirectTrajectoryWithCost(
//                start, sub_dest, initial_velocity, constraints, tree, obstacles);
//
//        for (double connection_time = SUB_DESTINATION_STEP_INTERVAL_SEC;
//             connection_time <= sub_trajectory.traj_path.getTotalTime();
//             connection_time += LAST_SUB_DESTINATION_STEP_INTERVAL_SEC)
//        {
//            // Copy the sub trajectory, then append a trajectory to the
//            // actual destination at connection_time
//            TrajectoryPath traj_path_to_dest = sub_trajectory.traj_path;
//            traj_path_to_dest.append(constraints, connection_time, destination);
//            TrajectoryPathWithCost full_traj_with_cost = getTrajectoryWithCost(
//                    traj_path_to_dest, tree, obstacles, sub_trajectory, connection_time);
//
//            // Incentivize following the last sub-destination
//            full_traj_with_cost.cost -= LAST_SUB_DESTINATION_BONUS;
//            if (full_traj_with_cost.cost< best_traj_with_cost.cost)
//            {
//                best_traj_with_cost = full_traj_with_cost;
//                last_sub_dest = sub_dest;
//            }
//
//            // Later connection_times will generally result in a longer trajectory duration,
//            // thus, if this trajectory does not have a collision, then we can not
//            // get a better trajectory with a later connection_time
//            // Alternatively, if this trajectory does collide and their exists a trajectory
//            // that doesn't collide and has a shorter duration, then we're not going to find
//            // a better trajectory by increasing the connection time.
//            if (!full_traj_with_cost.collides() ||
//                (!best_traj_with_cost.collides() &&
//                 full_traj_with_cost.traj_path.getTotalTime() > best_traj_with_cost.traj_path.getTotalTime()))
//            {
//                break;
//            }
//        }
//    }

    int num_traj = 1;
    // Add trajectories that go through sub-destinations
    for (const Point &sub_dest : sub_destinations)
    {
        TrajectoryPathWithCost sub_trajectory = getDirectTrajectoryWithCost(
                start, sub_dest, initial_velocity, constraints, tree, obstacles);

        for (double connection_time = SUB_DESTINATION_STEP_INTERVAL_SEC;
             connection_time <= sub_trajectory.traj_path.getTotalTime();
             connection_time += SUB_DESTINATION_STEP_INTERVAL_SEC)
        {
            // Copy the sub trajectory, then append a trajectory to the
            // actual destination at connection_time
            TrajectoryPath traj_path_to_dest = sub_trajectory.traj_path;
            traj_path_to_dest.append(constraints, connection_time, destination);
            // TODO: Shouldn't need to claculate cost everytime, can sometimes continue knowing that duration has increased
            TrajectoryPathWithCost full_traj_with_cost = getTrajectoryWithCost(
                    traj_path_to_dest, tree, obstacles, sub_trajectory, connection_time);
            num_traj++;
            if (full_traj_with_cost.cost < best_traj_with_cost.cost)
            {
                best_traj_with_cost = full_traj_with_cost;
                last_sub_dest = sub_dest;
            }

            // Later connection_times will generally result in a longer trajectory duration,
            // thus, if this trajectory does not have a collision, then we can not
            // get a better trajectory with a later connection_time
            // Alternatively, if this trajectory does collide and their exists a trajectory
            // that doesn't collide and has a shorter duration, then we're not going to find
            // a better trajectory by increasing the connection time.
            if (!full_traj_with_cost.collides() ||
                (!best_traj_with_cost.collides() &&
                full_traj_with_cost.traj_path.getTotalTime() > best_traj_with_cost.traj_path.getTotalTime()))
            {
                break;
            }
        }
    }

//    LOG(DEBUG) << "Best traj found with cost: " << best_traj_with_cost.cost << " num_traj: " << num_traj << " which has collision? " << best_traj_with_cost.collides();

    // TODO: Added for debugging
    auto end_time = std::chrono::high_resolution_clock::now();
    total_time +=
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time)
            .count();

    num_calls++;
    if (num_calls % 1000 == 0)
    {
        std::cout << "Average findTrajectory time (ignoring direct trajs): " << total_time / num_calls << "us"
                  << std::endl;
        total_time = 0;
        num_calls  = 0;
    }

    return best_traj_with_cost.traj_path;
}

TrajectoryPathWithCost TrajectoryPlanner::getDirectTrajectoryWithCost(
    const Point &start, const Point &destination, const Vector &initial_velocity,
    const KinematicConstraints &constraints, aabb::Tree &obstacle_tree,
    const std::vector<ObstaclePtr> &obstacles)
{
    // TODO: Consider removing this function and just using getTrajectoryWithCost
    return getTrajectoryWithCost(
        TrajectoryPath(std::make_shared<BangBangTrajectory2D>(
            start, destination, initial_velocity, constraints.getMaxVelocity(),
            constraints.getMaxAcceleration(), constraints.getMaxDeceleration()),
           [](const KinematicConstraints &constraints,
              const Point &initial_pos,
              const Point &final_pos,
              const Vector &initial_vel) {
            return std::make_shared<BangBangTrajectory2D>(
                    initial_pos, final_pos, initial_vel, constraints.getMaxVelocity(),
                    constraints.getMaxAcceleration(), constraints.getMaxDeceleration());
        }),
        obstacle_tree, obstacles, std::nullopt, std::nullopt);
}

TrajectoryPathWithCost TrajectoryPlanner::getTrajectoryWithCost(
    const TrajectoryPath &trajectory, aabb::Tree &obstacle_tree,
    const std::vector<ObstaclePtr> &obstacles,
    const std::optional<TrajectoryPathWithCost> &sub_traj_with_cost,
    const std::optional<double> sub_traj_duration_sec)
{
    TrajectoryPathWithCost traj_with_cost(trajectory);

    std::set<unsigned int> possible_collisions_indices;
    for (const BoundingBox &bounding_box : trajectory.getBoundingBoxes())
    {
        std::vector<unsigned int> bb_collisions =
            obstacle_tree.query(aabb::AABB({bounding_box.xMin(), bounding_box.yMin()},
                                           {bounding_box.xMax(), bounding_box.yMax()}));
        possible_collisions_indices.insert(bb_collisions.begin(), bb_collisions.end());
    }

    const double search_end_time_s = std::min(trajectory.getTotalTime(), MAX_FUTURE_COLLISION_CHECK_SEC);

    // Find the duration before the trajectory leaves all obstacles
    double first_non_collision_time;
    // Avoid finding the first non-collision time if the cache sub-trajectory
    // has a collision time.
    if (sub_traj_with_cost.has_value() &&
        sub_traj_with_cost->collision_duration_front_s < sub_traj_duration_sec)
    {
        first_non_collision_time = sub_traj_with_cost->collision_duration_front_s;
    }
    else
    {
        first_non_collision_time =
                getFirstNonCollisionTime(trajectory, possible_collisions_indices, obstacles, search_end_time_s);
    }
    traj_with_cost.collision_duration_front_s = first_non_collision_time;

    // TODO: Should fill colliding_obstacle ptr if it collides

    // Find the duration we're within an obstacle before search_end_time_s
    double last_non_collision_time =
            getLastNonCollisionTime(trajectory, possible_collisions_indices, obstacles,
                                    search_end_time_s);
    traj_with_cost.collision_duration_back_s = search_end_time_s - last_non_collision_time;

    // Get the first collision time, excluding the time at the start and end of path
    // that we may be in an obstacle for.
    if (sub_traj_with_cost.has_value() &&
        sub_traj_with_cost->first_collision_time_s < sub_traj_duration_sec)
    {
        traj_with_cost.first_collision_time_s = sub_traj_with_cost->first_collision_time_s;
        traj_with_cost.colliding_obstacle   = sub_traj_with_cost->colliding_obstacle;
    }
    else
    {
        std::pair<double, ObstaclePtr> collision =
            getFirstCollisionTime(trajectory, possible_collisions_indices, obstacles,
                                  first_non_collision_time, last_non_collision_time); // TODO: Can we limit start based on sub_traj_duration_sec?
        traj_with_cost.first_collision_time_s = collision.first;
        traj_with_cost.colliding_obstacle   = collision.second;
    }

    // TODO: Add collision_penalty?! Tiger's does it for defense area based on how much it
    // goes into it

    traj_with_cost.cost = calculateCost(traj_with_cost);

    return traj_with_cost;
}

double TrajectoryPlanner::calculateCost(
    const TrajectoryPathWithCost &traj_with_cost) const
{
    double total_cost = traj_with_cost.traj_path.getTotalTime();

    // TODO: Tiger's does this dynamically?! collisionPenalty. Mainly for penalty area
    if (traj_with_cost.colliding_obstacle != nullptr)
    {
        total_cost += 6.0;
        // std::cout << "Colliding with obstacle: " <<
        // traj_with_cost.colliding_obstacle->toString() << " ";
    }

    Point first_collision_position =
        traj_with_cost.traj_path.getPosition(traj_with_cost.first_collision_time_s);
    Point destination = traj_with_cost.traj_path.getDestination();
    total_cost += (first_collision_position - destination).length();
    // std::cout << " + (first_collision_position - destination).length(): " <<
    // (first_collision_position - destination).length() << " ";

    total_cost += std::max(
        0.0, (MAX_FUTURE_COLLISION_CHECK_SEC - traj_with_cost.first_collision_time_s));
    // std::cout << " + std::max(0.0, (MAX_FUTURE_COLLISION_CHECK -
    // traj_with_cost.first_collision_time)): " << std::max(0.0,
    // (MAX_FUTURE_COLLISION_CHECK - traj_with_cost.first_collision_time)) << " ";

    total_cost += 3 * traj_with_cost.collision_duration_front_s;
    // std::cout << " + 3 * traj_with_cost.collision_duration_front: " << 3 *
    // traj_with_cost.collision_duration_front << " ";

    total_cost += 1 * traj_with_cost.collision_duration_back_s;
    // std::cout << " + 1 * traj_with_cost.collision_duration_back: " << 1 *
    // traj_with_cost.collision_duration_back << " = " << total_cost << std::endl;

    return total_cost;
}

double TrajectoryPlanner::getFirstNonCollisionTime(const TrajectoryPath &traj_path,
                                                   const std::set<unsigned int> &obstacle_indices,
                                                   const std::vector<ObstaclePtr> &obstacles,
                                                   const double search_end_time_s) const
{
    double path_length = traj_path.getTotalTime();
    for (double time = 0.0; time <= search_end_time_s;
         time += FORWARD_COLLISION_CHECK_STEP_INTERVAL_SEC)
    {
        Point position       = traj_path.getPosition(time);
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

std::pair<double, ObstaclePtr> TrajectoryPlanner::getFirstCollisionTime(
    const TrajectoryPath &traj_path, const std::set<unsigned int> &obstacle_indices,
    const std::vector<ObstaclePtr> &obstacles, const double start_time_sec,
    const double search_end_time_s) const
{
    for (double time = start_time_sec;
         time <= search_end_time_s;
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

double TrajectoryPlanner::getLastNonCollisionTime(const TrajectoryPath &traj_path,
                                                  const std::set<unsigned int> &obstacle_indices,
                                                  const std::vector<ObstaclePtr> &obstacles,
                                                  const double search_end_time_s) const
{
    for (double time = search_end_time_s; time >= 0.0;
         time -= COLLISION_CHECK_STEP_INTERVAL_SEC)
    {
        Point position       = traj_path.getPosition(time);
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
    return search_end_time_s;
}

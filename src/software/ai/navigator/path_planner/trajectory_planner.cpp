#include "software/ai/navigator/path_planner/trajectory_planner.h"

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

    std::vector<TrajectoryPath> possible_paths;
    possible_paths.reserve(sub_destinations.size() + 1);

    // Add a default trajectory from start to destination
    possible_paths.emplace_back(BangBangTrajectory2D(
        start, destination, initial_velocity, constraints.getMaxVelocity(),
        constraints.getMaxAcceleration(), constraints.getMaxDeceleration()));

    // Add trajectories that go through sub-destinations
    for (const Point &sub_dest : sub_destinations)
    {
        TrajectoryPath trajectory_path(BangBangTrajectory2D(
            start, sub_dest, initial_velocity, constraints.getMaxVelocity(),
            constraints.getMaxAcceleration(), constraints.getMaxDeceleration()));
        for (Duration connection_time = SUB_DESTINATION_STEP_INTERVAL;
             connection_time < trajectory_path.getTotalTime();
             connection_time += SUB_DESTINATION_STEP_INTERVAL)
        {
            possible_paths.push_back(trajectory_path);
            possible_paths.back().append(constraints, connection_time, destination);
        }
    }

    // TODO: This can probably be shared between all findTrajectory calls in the same
    // tick.
    //       Should create a wrapper which automatically creates this (and hides it) and
    //       stores the obstacles
    aabb::Tree tree(2, 0.0, {false, false},
                    {navigable_area.xLength(), navigable_area.yLength()},
                    static_cast<unsigned int>(obstacles.size()), false);
    for (unsigned int i = 0; i < obstacles.size(); i++)
    {
        Rectangle aabb         = obstacles[i]->axisAlignedBoundingBox();
        std::vector aabb_lower = {aabb.negXNegYCorner().x(), aabb.negXNegYCorner().y()};
        std::vector aabb_upper = {aabb.posXPosYCorner().x(), aabb.posXPosYCorner().y()};
        tree.insertParticle(i, aabb_lower, aabb_upper);
    }

    // Find the trajectory with the lowest cost
    double lowest_cost       = std::numeric_limits<double>::infinity();
    size_t lowest_cost_index = 0;
    for (size_t i = 0; i < possible_paths.size(); ++i)
    {
        double cost = calculateCost(possible_paths[i], tree, obstacles);
        if (cost < lowest_cost)
        {
            lowest_cost       = cost;
            lowest_cost_index = i;
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    total_time +=
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time)
            .count();

    num_calls++;
    if (num_calls % 1000 == 0)
    {
        std::cout << "Average total time: " << total_time / num_calls << "us"
                  << std::endl;
        total_time = 0;
        num_calls  = 0;
    }

    return possible_paths[lowest_cost_index];
}

double TrajectoryPlanner::calculateCost(const TrajectoryPath &trajectory_path,
                                        aabb::Tree &obstacle_tree,
                                        const std::vector<ObstaclePtr> &obstacles)
{
    double cost = trajectory_path.getTotalTime().toSeconds();

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
    Duration earliest_collision_time =
        trajectory_path.getTotalTime() + Duration::fromSeconds(1);
    for (Duration time = Duration(); time < trajectory_path.getTotalTime() && time < MAX_FUTURE_COLLISION_CHECK;
         time += COLLISION_CHECK_STEP_INTERVAL)
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
        cost += (trajectory_path.getTotalTime() - earliest_collision_time).toSeconds();
    }

    if (starts_in_collision)
    {
        // TODO;
        cost += 0.0;
    }

    return cost;
}

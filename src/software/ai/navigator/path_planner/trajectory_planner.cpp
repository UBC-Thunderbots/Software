#include "software/ai/navigator/path_planner/trajectory_planner.h"

TrajectoryPlanner::TrajectoryPlanner()
{
    // Initialize the relative sub-destinations array
    const Angle sub_angles = Angle::full() / NUM_SUB_DESTINATION_ANGLES;
    for (const double distance : SUB_DESTINATION_DISTANCES_METERS)
    {
        for (int i = 0; i < NUM_SUB_DESTINATION_ANGLES; ++i)
        {
            Angle angle = sub_angles * i;
            relative_sub_destinations.emplace_back(Vector::createFromAngle(angle).normalize(distance));
        }
    }
}

std::optional<TrajectoryPath>
TrajectoryPlanner::findTrajectory(const Point &start, const Point &destination, const Vector &initial_velocity,
                                  const KinematicConstraints &constraints, const std::vector<ObstaclePtr> &obstacles,
                                  const Rectangle &navigable_area)
{
    std::vector<Point> sub_destinations(relative_sub_destinations.size());
    for (const Vector& relative_sub_dest : relative_sub_destinations)
    {
        Point sub_dest = start + relative_sub_dest;
        if (contains(navigable_area, sub_dest))
        {
            sub_destinations.emplace_back(sub_dest);
        }
    }

    std::vector<TrajectoryPath> possible_paths;
    possible_paths.reserve(sub_destinations.size() + 1);

    // TODO:
    // Generate trajectory to (sub destination then to) destination
    // Generate cost for trajectory
    // Return trajectory with the lowest cost

    // Add a default trajectory from start to destination
    possible_paths.emplace_back(BangBangTrajectory2D(start, destination, initial_velocity, constraints.getMaxVelocity(), constraints.getMaxAcceleration(), constraints.getMaxDeceleration()));

    // Add trajectories that go through sub-destinations
    for (const Point& sub_dest : sub_destinations)
    {
        TrajectoryPath trajectory_path(BangBangTrajectory2D(start, sub_dest, initial_velocity, constraints.getMaxVelocity(), constraints.getMaxAcceleration(), constraints.getMaxDeceleration()));
        for (Duration connection_time = SUB_DESTINATION_STEP_INTERVAL; connection_time < trajectory_path.getTotalTime(); connection_time += SUB_DESTINATION_STEP_INTERVAL)
        {
            possible_paths.push_back(trajectory_path);
            possible_paths.end()->append(constraints, connection_time, destination);
        }
    }



    return std::optional<TrajectoryPath>();
}

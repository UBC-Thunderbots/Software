#pragma once

#include <optional>
#include "software/ai/navigator/path_planner/trajectory_path.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"

class TrajectoryPlanner
{
public:
    TrajectoryPlanner();

    std::optional<TrajectoryPath>
    findTrajectory(const Point &start, const Point &destination, const Vector &initial_velocity,
                   const KinematicConstraints &constraints, const std::vector<ObstaclePtr> &obstacles,
                   const Rectangle &navigable_area);

private:

    std::vector<Vector> relative_sub_destinations;

    static constexpr std::array<double, 4> SUB_DESTINATION_DISTANCES_METERS = {0.1, 1, 2, 3};
    static constexpr unsigned int NUM_SUB_DESTINATION_ANGLES = 15;
    const Duration SUB_DESTINATION_STEP_INTERVAL = Duration::fromSeconds(0.2);
};
#pragma once

#include "software/ai/navigator/path_planner/trajectory_path.h"

class TrajectoryPathWithCost
{
public:
    TrajectoryPathWithCost() = delete;

    TrajectoryPathWithCost(const TrajectoryPath& traj_path, double cost);

private:
    TrajectoryPath traj_path;
    double cost;
};

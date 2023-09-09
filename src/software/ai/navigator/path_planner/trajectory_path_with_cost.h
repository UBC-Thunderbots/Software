#pragma once

#include "software/ai/navigator/path_planner/trajectory_path.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"

class TrajectoryPathWithCost
{
   public:
    TrajectoryPathWithCost() = delete;

    TrajectoryPathWithCost(const TrajectoryPath& traj_path, double cost = 0.0);

    bool collides() const;


    TrajectoryPath traj_path;
    double cost;
    // The duration before the trajectory leaves an obstacle that it starts
    // within. 0 if the trajectory does not start within an obstacle.
    Duration collision_duration_front;
    // The duration we're within an obstacle before the trajectory end.
    // Equals duration of trajectory if it does not end in an obstacle.
    Duration collision_duration_back;
    Duration first_collision_time;
    Duration collision_lookahead;
    ObstaclePtr colliding_obstacle;
};

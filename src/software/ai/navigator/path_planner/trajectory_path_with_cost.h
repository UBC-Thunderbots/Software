#pragma once

#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/ai/navigator/path_planner/trajectory_path.h"

class TrajectoryPathWithCost
{
   public:
    TrajectoryPathWithCost() = delete;

    TrajectoryPathWithCost(const TrajectoryPath& traj_path, double cost = 0.0);

    /**
     * Returns true if the trajectory collides with an obstacle
     *
     * @return true if the trajectory collides with an obstacle
     */
    bool collides() const;


    TrajectoryPath traj_path;
    double cost = 0.0;
    // The duration before the trajectory leaves an obstacle that it starts
    // within. 0 if the trajectory does not start within an obstacle.
    double collision_duration_front_s = 0.0;
    // The duration we're within an obstacle before the trajectory end.
    // Equals duration of trajectory if it does not end in an obstacle.
    double collision_duration_back_s = 0.0;
    double first_collision_time_s    = 0.0;
    double collision_lookahead       = 0.0;
    ObstaclePtr colliding_obstacle   = nullptr;
};

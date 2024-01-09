#pragma once

#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/ai/navigator/trajectory/trajectory_path.h"

/**
 * A wrapper around TrajectoryPath for holding additional information about the
 * cost of the trajectory path
 */
class TrajectoryPathWithCost
{
   public:
    TrajectoryPathWithCost() = delete;

    explicit TrajectoryPathWithCost(const TrajectoryPath& traj_path);

    /**
     * Returns true if the trajectory collides with an obstacle
     *
     * @return true if the trajectory collides with an obstacle
     */
    bool collides() const;

    // The trajectory path that the costs are associated with
    TrajectoryPath traj_path;

    // The duration before the trajectory leaves an obstacle that it starts
    // within. 0 if the trajectory does not start within an obstacle.
    double collision_duration_front_s = 0.0;

    // The duration we're within an obstacle before the trajectory end.
    // 0 if the trajectory does not end within an obstacle.
    double collision_duration_back_s = 0.0;

    // The time and obstacle at which the trajectory first collides with an obstacle
    // first_collision_time_s is set to infinity and colliding_obstacle is nullptr
    // if the trajectory does not collide.
    // Note that collisions that the trajectory starts or ends in are not considered as
    // those are unavoidable + they are handled by collision_duration_front_s and
    // collision_duration_back_s.
    double first_collision_time_s  = std::numeric_limits<double>::max();
    ObstaclePtr colliding_obstacle = nullptr;

    // Total cost of the trajectory path
    double cost = 0.0;
};

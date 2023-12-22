#include "software/ai/navigator/trajectory/trajectory_path_with_cost.h"

TrajectoryPathWithCost::TrajectoryPathWithCost(const TrajectoryPath &traj_path,
                                               double cost)
    : traj_path(traj_path), cost(cost)
{
}

bool TrajectoryPathWithCost::collides() const
{
    return colliding_obstacle != nullptr ||
           first_collision_time_s < traj_path.getTotalTime() ||
           collision_duration_front_s > 0 || collision_duration_back_s > 0;
}

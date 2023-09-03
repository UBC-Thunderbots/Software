#pragma once

#include "software/ai/navigator/path_planner/bang_bang_trajectory_2d.h"

// TODO: Consider making this a private class in trajectory path
class TrajectoryPathNode
{
   public:
    TrajectoryPathNode(const BangBangTrajectory2D &trajectory,
                       Duration trajectory_end_time)
        : trajectory(trajectory), trajectory_end_time(trajectory_end_time){};

    TrajectoryPathNode(const BangBangTrajectory2D &trajectory)
        : trajectory(trajectory), trajectory_end_time(trajectory.getTotalTime()){};

    const BangBangTrajectory2D &getTrajectory() const
    {
        return trajectory;
    }

    const Duration &getTrajectoryEndTime() const
    {
        return trajectory_end_time;
    }

    void setTrajectoryEndTime(const Duration &new_end_time)
    {
        trajectory_end_time = new_end_time;
    }

   private:
    // TODO: Consider making this a pointer of 2D trajectories so the traj planner can be
    // replaced
    BangBangTrajectory2D trajectory;
    Duration trajectory_end_time;
};

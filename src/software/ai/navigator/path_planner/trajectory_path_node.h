#pragma once

#include "software/ai/navigator/path_planner/bang_bang_trajectory_2d.h"

// TODO: Consider making this a private class in trajectory path
class TrajectoryPathNode
{
   public:
    TrajectoryPathNode(const BangBangTrajectory2D &trajectory,
                       double trajectory_end_time)
        : trajectory(trajectory), trajectory_end_time_sec(trajectory_end_time){};

    TrajectoryPathNode(const BangBangTrajectory2D &trajectory)
        : trajectory(trajectory), trajectory_end_time_sec(trajectory.getTotalTime()){};

    const BangBangTrajectory2D &getTrajectory() const
    {
        return trajectory;
    }

    const double &getTrajectoryEndTime() const
    {
        return trajectory_end_time_sec;
    }

    void setTrajectoryEndTime(const double &new_end_time_sec)
    {
        trajectory_end_time_sec = new_end_time_sec;
    }

   private:
    // TODO: Consider making this a pointer of 2D trajectories so the traj planner can be
    // replaced
    BangBangTrajectory2D trajectory;
    double trajectory_end_time_sec;
};

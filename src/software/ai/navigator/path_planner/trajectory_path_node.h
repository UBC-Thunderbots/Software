#pragma once

#include "software/ai/navigator/path_planner/bang_bang_trajectory_2d.h"

// TODO: Consider making this a private class in trajectory path
class TrajectoryPathNode
{
   public:
    /**
     * Constructor for a trajectory and its end time
     * @param trajectory Trajectory of this trajectory path node
     * @param trajectory_end_time End time of this trajectory
     */
    TrajectoryPathNode(const BangBangTrajectory2D &trajectory, double trajectory_end_time)
        : trajectory(trajectory), trajectory_end_time_sec(trajectory_end_time){};

    /**
     * Constructor for a trajectory. It is assumed that the end time
     * is the total time of the trajectory
     * @param trajectory Trajectory of this trajectory path node
     */
    TrajectoryPathNode(const BangBangTrajectory2D &trajectory)
        : trajectory(trajectory), trajectory_end_time_sec(trajectory.getTotalTime()){};

    /**
     * Get the trajectory of this trajectory path node
     * @return Trajectory of this trajectory path node
     */
    const BangBangTrajectory2D &getTrajectory() const
    {
        return trajectory;
    }

    /**
     * Get the end time of this trajectory
     * @return The end time of this trajectory
     */
    const double &getTrajectoryEndTime() const
    {
        return trajectory_end_time_sec;
    }

    /**
     * Update the end time of this trajectory
     * @param new_end_time_sec New end time
     */
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

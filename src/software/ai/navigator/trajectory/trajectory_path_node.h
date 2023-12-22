#pragma once

#include <memory>

#include "software/ai/navigator/trajectory/bang_bang_trajectory_2d.h"

/**
 * A class that wraps Trajectory2D and allows for earlier trajectory
 * end-time than the actual full duration. This is useful for having
 * multiple trajectories continuously connected to each other to form
 * a path (TrajectoryPath).
 */
class TrajectoryPathNode
{
   public:
    /**
     * Constructor for a trajectory and its end time
     * @param trajectory Trajectory of this trajectory path node
     * @param trajectory_end_time End time of this trajectory
     */
    TrajectoryPathNode(const std::shared_ptr<Trajectory2D> &trajectory,
                       double trajectory_end_time)
        : trajectory(trajectory), trajectory_end_time_sec(trajectory_end_time){};

    /**
     * Constructor for a trajectory. It is assumed that the end time
     * is the total time of the trajectory
     * @param trajectory Trajectory of this trajectory path node
     */
    TrajectoryPathNode(const std::shared_ptr<Trajectory2D> &trajectory)
        : trajectory(trajectory), trajectory_end_time_sec(trajectory->getTotalTime()){};

    /**
     * Get the trajectory of this trajectory path node
     * @return Trajectory of this trajectory path node
     */
    const std::shared_ptr<Trajectory2D> &getTrajectory() const
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
    std::shared_ptr<Trajectory2D> trajectory;
    double trajectory_end_time_sec;
};

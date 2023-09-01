#include "software/ai/navigator/path_planner/trajectory_path.h"

Point TrajectoryPath::getPosition(Duration t) const
{
    for (const TrajectoryPathNode& traj : traj_path)
    {
        if (t <= traj.getTrajectoryEndTime())
        {
            return traj.getTrajectory().getPosition(t);
        }
        else
        {
            t -= traj.getTrajectoryEndTime();
        }
    }

    return Point();
}

Vector TrajectoryPath::getVelocity(Duration t) const
{
    for (const TrajectoryPathNode& traj : traj_path)
    {
        if (t <= traj.getTrajectoryEndTime())
        {
            return traj.getTrajectory().getVelocity(t);
        }
        else
        {
            t -= traj.getTrajectoryEndTime();
        }
    }

    return Vector();
}

Vector TrajectoryPath::getAcceleration(Duration t) const
{
    for (const TrajectoryPathNode& traj : traj_path)
    {
        if (t <= traj.getTrajectoryEndTime())
        {
            return traj.getTrajectory().getAcceleration(t);
        }
        else
        {
            t -= traj.getTrajectoryEndTime();
        }
    }

    return Vector();
}

Duration TrajectoryPath::getTotalTime() const
{
    Duration total_time;
    for (const TrajectoryPathNode& traj : traj_path)
    {
        total_time += traj.getTrajectoryEndTime();
    }
    return total_time;
}
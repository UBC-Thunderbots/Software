#include "software/ai/navigator/path_planner/trajectory_path.h"
#include "software/logger/logger.h"

TrajectoryPath::TrajectoryPath(const BangBangTrajectory2D &initial_trajectory)
        : traj_path({TrajectoryPathNode(initial_trajectory)})
{}

void TrajectoryPath::append(const KinematicConstraints &constraints, Duration connection_time, const Point& destination)
{
    for (int i = 0; i < traj_path.size(); i++)
    {
        if (connection_time <= traj_path[i].getTrajectoryEndTime())
        {
            traj_path[i].setTrajectoryEndTime(connection_time);

            // Delete all trajectory nodes after the on that is being connected to
            traj_path.erase(traj_path.begin() + i + 1, traj_path.end());

            Point connection_pos = getPosition(connection_time);
            Vector connection_vel = getVelocity(connection_time);
            BangBangTrajectory2D child_traj;
            child_traj.generate(connection_pos, destination, connection_vel, constraints.getMaxVelocity(), constraints.getMaxAcceleration(), constraints.getMaxDeceleration());
            traj_path.emplace_back(child_traj);
            return;
        }
        else
        {
            connection_time -= traj_path[i].getTrajectoryEndTime();
        }
    }

    LOG(FATAL) << "TrajectoryPath::append called with connection_time > getTotalTime() = " << getTotalTime() << " (Num trajectories already in path: " << traj_path.size() << ")";
}

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


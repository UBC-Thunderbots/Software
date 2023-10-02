#include "software/ai/navigator/path_planner/trajectory_path.h"

#include "software/logger/logger.h"

TrajectoryPath::TrajectoryPath(const BangBangTrajectory2D& initial_trajectory)
    : traj_path({TrajectoryPathNode(initial_trajectory)})
{
}

void TrajectoryPath::append(const KinematicConstraints& constraints,
                            double connection_time_sec, const Point& destination)
{
    for (size_t i = 0; i < traj_path.size(); i++)
    {
        if (connection_time_sec <= traj_path[i].getTrajectoryEndTime())
        {
            traj_path[i].setTrajectoryEndTime(connection_time_sec);

            // Delete all trajectory nodes after the on that is being connected to
            traj_path.erase(traj_path.begin() + i + 1, traj_path.end());

            Point connection_pos  = getPosition(connection_time_sec);
            Vector connection_vel = getVelocity(connection_time_sec);
            BangBangTrajectory2D child_traj;
            child_traj.generate(
                connection_pos, destination, connection_vel, constraints.getMaxVelocity(),
                constraints.getMaxAcceleration(), constraints.getMaxDeceleration());
            traj_path.emplace_back(child_traj);
            return;
        }
        else
        {
            connection_time_sec -= traj_path[i].getTrajectoryEndTime();
        }
    }

    LOG(FATAL) << "TrajectoryPath::append called with connection_time > getTotalTime() = "
               << getTotalTime()
               << " (Num trajectories already in path: " << traj_path.size() << ")";
}

Point TrajectoryPath::getPosition(double t_sec) const
{
    for (const TrajectoryPathNode& traj : traj_path)
    {
        if (t_sec <= traj.getTrajectoryEndTime())
        {
            return traj.getTrajectory().getPosition(t_sec);
        }
        else
        {
            t_sec -= traj.getTrajectoryEndTime();
        }
    }

    return traj_path.back().getTrajectory().getDestination();
}

Vector TrajectoryPath::getVelocity(double t_sec) const
{
    for (const TrajectoryPathNode& traj : traj_path)
    {
        if (t_sec <= traj.getTrajectoryEndTime())
        {
            return traj.getTrajectory().getVelocity(t_sec);
        }
        else
        {
            t_sec -= traj.getTrajectoryEndTime();
        }
    }

    return Vector();
}

Vector TrajectoryPath::getAcceleration(double t_sec) const
{
    for (const TrajectoryPathNode& traj : traj_path)
    {
        if (t_sec <= traj.getTrajectoryEndTime())
        {
            return traj.getTrajectory().getAcceleration(t_sec);
        }
        else
        {
            t_sec -= traj.getTrajectoryEndTime();
        }
    }

    return Vector();
}

double TrajectoryPath::getTotalTime() const
{
    double total_time = 0.0;
    for (const TrajectoryPathNode& traj : traj_path)
    {
        total_time += traj.getTrajectoryEndTime();
    }
    return total_time;
}

std::vector<Rectangle> TrajectoryPath::getBoundingBoxes() const
{
    std::vector<Rectangle> bounding_boxes;
    for (const TrajectoryPathNode& traj_node : traj_path)
    {
        bounding_boxes.push_back(traj_node.getTrajectory().getBoundingBox());
    }
    return bounding_boxes;
}

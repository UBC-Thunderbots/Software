#include "software/ai/navigator/path_planner/trajectory_path.h"

#include "software/logger/logger.h"

TrajectoryPath::TrajectoryPath(const std::shared_ptr<Trajectory2D>& initial_trajectory,
                               std::function<std::shared_ptr<Trajectory2D>(const KinematicConstraints& constraints,
                                                                           const Point& initial_pos,
                                                                           const Point& final_pos,
                                                                           const Vector& initial_vel)> traj_generator)
    : traj_path({TrajectoryPathNode(initial_trajectory)}),
      trajectory_generator(traj_generator)
{
}

void TrajectoryPath::append(const KinematicConstraints& constraints,
                            double connection_time_sec, const Point& destination)
{
    // Find the trajectory path node that the new trajectory should connect to
    for (size_t i = 0; i < traj_path.size(); i++)
    {
        if (connection_time_sec <= traj_path[i].getTrajectoryEndTime())
        {
            traj_path[i].setTrajectoryEndTime(connection_time_sec);

            // Delete all trajectory nodes after the on that is being connected to
            traj_path.erase(traj_path.begin() + i + 1, traj_path.end());

            Point connection_pos  = getPosition(connection_time_sec);
            Vector connection_vel = getVelocity(connection_time_sec);
            traj_path.emplace_back(trajectory_generator(constraints, connection_pos, destination, connection_vel));
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
            return traj.getTrajectory()->getPosition(t_sec);
        }
        else
        {
            t_sec -= traj.getTrajectoryEndTime();
        }
    }

    return traj_path.back().getTrajectory()->getDestination();
}

Vector TrajectoryPath::getVelocity(double t_sec) const
{
    for (const TrajectoryPathNode& traj : traj_path)
    {
        if (t_sec <= traj.getTrajectoryEndTime())
        {
            return traj.getTrajectory()->getVelocity(t_sec);
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
            return traj.getTrajectory()->getAcceleration(t_sec);
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

std::vector<BoundingBox> TrajectoryPath::getBoundingBoxes() const
{
    std::vector<BoundingBox> bounding_boxes;
    for (const TrajectoryPathNode& traj_node : traj_path)
    {
        const std::vector<BoundingBox> bbs = traj_node.getTrajectory()->getBoundingBoxes();
        bounding_boxes.insert(bounding_boxes.begin(), bbs.begin(), bbs.end());
    }
    return bounding_boxes;
}

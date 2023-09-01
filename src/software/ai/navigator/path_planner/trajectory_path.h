#pragma once

#include "software/ai/navigator/path_planner/trajectory_path_node.h"
#include "software/ai/navigator/path_planner/kinematic_constraints.h"

/**
 * TrajectoryPath represents a list of 2D trajectories that are connected end-to-end
 * to form a path. A TrajectoryNode is a 2D trajectory and the time at which it ends
 * and the next TrajectoryNode begins.
 */
class TrajectoryPath : public Trajectory<Point, Vector, Vector>
{
public:
    TrajectoryPath() = delete;

    TrajectoryPath(const BangBangTrajectory2D& initial_trajectory);

    void append(const KinematicConstraints& constraints, Duration connection_time, const Point& destination);

    /**
     * Get the position at time t of this trajectory path
     *
     * @param t The time elapsed since the start of the trajectory path
     * @return The position at time t
     */
    Point getPosition(Duration t) const override;

    /**
     * Get the velocity at time t of this trajectory path
     *
     * @param t The time elapsed since the start of the trajectory path
     * @return The velocity at time t
     */
    Vector getVelocity(Duration t) const override;

    /**
     * Get the acceleration at time t of this trajectory path
     *
     * @param t The time elapsed since the start of the trajectory path
     * @return The acceleration at time t
     */
    Vector getAcceleration(Duration t) const override;

    /**
     * Get the total duration of the trajectory until it reaches the destination
     *
     * @return The total duration for this trajectory path
     */
    Duration getTotalTime() const override;

private:
    std::vector<TrajectoryPathNode> traj_path;
};

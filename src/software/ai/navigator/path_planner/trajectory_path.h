#pragma once

#include <memory>
#include <functional>
#include "software/ai/navigator/path_planner/kinematic_constraints.h"
#include "software/ai/navigator/path_planner/trajectory_path_node.h"

/**
 * TrajectoryPath represents a list of 2D trajectories that are connected end-to-end
 * to form a path. A TrajectoryNode is a 2D trajectory and the time at which it ends
 * and the next TrajectoryNode begins.
 */
class TrajectoryPath : public Trajectory2D
{
   public:
    TrajectoryPath() = delete;

    /**
     * Constructor
     *
     * @param initial_trajectory The initial trajectory of this trajectory path
     * @param traj_generator A function that generates a new trajectory given the
     * kinematic constraints, initial position, and final position
     */
    TrajectoryPath(const std::shared_ptr<Trajectory2D>& initial_trajectory,
                   std::function<std::shared_ptr<Trajectory2D>(const KinematicConstraints& constraints,
                                                               const Point& initial_pos,
                                                               const Point& final_pos,
                                                               const Vector& initial_vel)> traj_generator);

    /**
     * Generate and append a new trajectory to the end of this trajectory path
     *
     * @param constraints Constraints of the new generated trajectory
     * @param connection_time_sec The time where the last existing trajectory should
     * connect to the newly generated trajectory
     * @param destination Destination of the newly generated trajectory
     */
    void append(const KinematicConstraints& constraints, double connection_time_sec,
                const Point& destination);

    /**
     * Get the position at time t of this trajectory path
     *
     * @param t The time elapsed since the start of the trajectory path
     * @return The position at time t
     */
    Point getPosition(double t_sec) const override;

    /**
     * Get the velocity at time t of this trajectory path
     *
     * @param t The time elapsed since the start of the trajectory path
     * @return The velocity at time t
     */
    Vector getVelocity(double t_sec) const override;

    /**
     * Get the acceleration at time t of this trajectory path
     *
     * @param t The time elapsed since the start of the trajectory path
     * @return The acceleration at time t
     */
    Vector getAcceleration(double t_sec) const override;

    /**
     * Get the total duration of the trajectory until it reaches the destination
     *
     * @return The total duration for this trajectory path
     */
    double getTotalTime() const override;

    /**
     * Get the bounding boxes of the trajectory path
     * @return A list of bounding boxes which wrap this trajectory path
     */
    std::vector<BoundingBox> getBoundingBoxes() const override;

   private:
    std::vector<TrajectoryPathNode> traj_path;
    std::function<std::shared_ptr<Trajectory2D>(const KinematicConstraints& constraints,
                                                const Point& initial_pos,
                                                const Point& final_pos,
                                                const Vector& initial_vel)>
        trajectory_generator;
};

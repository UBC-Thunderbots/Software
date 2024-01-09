#pragma once

#include <functional>
#include <memory>

#include "software/ai/navigator/trajectory/kinematic_constraints.h"
#include "software/ai/navigator/trajectory/trajectory_path_node.h"

using TrajectoryGenerator = std::function<std::shared_ptr<Trajectory2D>(
const Point& initial_pos,
    const Point& final_pos, const Vector& initial_vel,
    const KinematicConstraints& constraints)>;

/**
 * TrajectoryPath represents a list of 2D trajectories that are connected end-to-end
 * to form a path. A TrajectoryPathNode is a 2D trajectory and the time at which it ends
 * and the next TrajectoryPathNode begins.
 */
class TrajectoryPath : public Trajectory2D
{
   public:
    TrajectoryPath() = delete;

    /**
     * Constructor
     *
     * @param initial_trajectory The initial trajectory of this trajectory path
     * @param traj_generator A function used to generate new trajectories given the
     * kinematic constraints, initial position, final position, and initial velocity.
     */
    TrajectoryPath(const std::shared_ptr<Trajectory2D>& initial_trajectory,
                   const TrajectoryGenerator& traj_generator);

    /**
     * Generate and append a new trajectory to the end of this trajectory path
     *
     * @param constraints Constraints of the new generated trajectory
     * @param connection_time_sec The time where the last existing trajectory should
     * connect to the newly generated trajectory
     * @param destination Destination of the newly generated trajectory
     */
    void append(double connection_time_sec, const Point& destination,
                const KinematicConstraints& constraints);

    /**
     * Get the position at time t of this trajectory path
     *
     * @param t_sec The time elapsed since the start of the trajectory path
     * @return The position at time t
     */
    Point getPosition(double t_sec) const override;

    /**
     * Get the velocity at time t of this trajectory path
     *
     * @param t_sec The time elapsed since the start of the trajectory path
     * @return The velocity at time t
     */
    Vector getVelocity(double t_sec) const override;

    /**
     * Get the acceleration at time t of this trajectory path
     *
     * @param t_sec The time elapsed since the start of the trajectory path
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
    std::vector<Rectangle> getBoundingBoxes() const override;

    /**
     * Get the list of TrajectoryPathNodes that make up this trajectory path
     *
     * @return The list of TrajectoryPathNodes that make up this trajectory path
     */
    const std::vector<TrajectoryPathNode>& getTrajectoryPathNodes() const;

   private:
    std::vector<TrajectoryPathNode> traj_path;
    TrajectoryGenerator trajectory_generator;
};

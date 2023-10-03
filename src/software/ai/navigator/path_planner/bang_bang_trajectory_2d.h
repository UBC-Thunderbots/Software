#pragma once

#include "software/ai/navigator/path_planner/bang_bang_trajectory_1d.h"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"

using Trajectory2D = Trajectory<Point, Vector, Vector>;

class BangBangTrajectory2D : public Trajectory2D
{
   public:
    /**
     * Default constructor. Creates an empty trajectory.
     */
    BangBangTrajectory2D() = default;

    /**
     * Generate a 2D trajectory from the initial position to the final position with the
     * given initial velocity and kinematic constraints.
     *
     * @param initial_pos Starting position of the trajectory
     * @param final_pos Destination. Where the trajectory should end at
     * @param initial_vel The initial velocity of the trajectory
     * @param max_vel The max velocity (in 2D) that the trajectory can reach at any given
     * point
     * @param max_accel The max acceleration (in 2D) that the trajectory can reach at any
     * given point
     * @param max_decel The max deceleration (in 2D) that the trajectory can reach at any
     * given point
     */
    BangBangTrajectory2D(const Point& initial_pos, const Point& final_pos,
                         const Vector& initial_vel, double max_vel, double max_accel,
                         double max_decel);

    /**
     * Generate a 2D trajectory from the initial position to the final position with the
     * given initial velocity and kinematic constraints.
     * @note that calling generate will overwrite the existing trajectory.
     *
     * @param initial_pos Starting position of the trajectory
     * @param final_pos Destination. Where the trajectory should end at
     * @param initial_vel The initial velocity of the trajectory
     * @param max_vel The max velocity (in 2D) that the trajectory can reach at any given
     * point
     * @param max_accel The max acceleration (in 2D) that the trajectory can reach at any
     * given point
     * @param max_decel The max deceleration (in 2D) that the trajectory can reach at any
     * given point
     */
    void generate(const Point& initial_pos, const Point& final_pos,
                  const Vector& initial_vel, double max_vel, double max_accel,
                  double max_decel);

    /**
     * Get the position at time t
     *
     * @param t The time elapsed since the start of the trajectory in seconds
     * @return The position at time t
     */
    Point getPosition(double t_sec) const override;

    /**
     * Get the velocity at time t
     *
     * @param t The time elapsed since the start of the trajectory in seconds
     * @return The velocity at time t
     */
    Vector getVelocity(double t_sec) const override;

    /**
     * Get the acceleration at time t
     *
     * @param t The time elapsed since the start of the trajectory in seconds
     * @return The acceleration at time t
     */
    Vector getAcceleration(double t_sec) const override;

    /**
     * Get the total duration of the trajectory until it reaches the destination
     *
     * @return The total duration for the trajectory in seconds
     */
    double getTotalTime() const override;

    /**
     * Get the bounding box of the trajectory
     * @return bounding box which bounds the trajectory
     */
    Rectangle getBoundingBox() const;

   private:
    BangBangTrajectory1D x_trajectory;
    BangBangTrajectory1D y_trajectory;

    // The maximum difference the x and y trajectory runtimes could have from each other
    // in seconds
    static constexpr double TRAJ_ACCURACY_TOLERANCE_SEC = 0.01;
};

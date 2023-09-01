#pragma once

#include "software/ai/navigator/path_planner/bang_bang_trajectory_1d.h"
#include "software/geom/angle.h"
#include "software/geom/angular_velocity.h"
#include "software/geom/angular_acceleration.h"

class BangBangTrajectory1DAngular : public Trajectory<Angle, AngularVelocity, AngularAcceleration>
{
   public:
    BangBangTrajectory1DAngular() = default;

    /**
     * Generate a 1D trajectory from the initial orientation to the final orientation with the
     * given initial angular velocity and kinematic constraints.
     * @note The generated trajectory will be the shortest between the two orientations.
     * E.g. Rotating from 0 to 360 degrees will generate a trajectory with no rotation.
     * @note This method will overwrite the existing trajectory.
     * @note The generated trajectory will always have a final angular velocity of 0
     *
     * @param initial_orient Starting orientation of the trajectory
     * @param final_orient Destination. Where the trajectory should end at
     * @param initial_angular_vel The angular velocity at the start of the trajectory
     * @param relative_final_orient The maximum angular velocity (magnitude) the trajectory could have
     * @param max_angular_accel The maximum angular acceleration the trajectory could have
     * @param max_angular_decel The maximum deceleration the trajectory could have
     */
    void generate(Angle initial_orient, Angle final_orient, AngularVelocity initial_angular_vel,
                  AngularVelocity max_angular_vel, AngularAcceleration max_angular_accel, AngularAcceleration max_angular_decel);

    /**
     * Get orientation at time t
     *
     * @param t Duration elapsed since start of trajectory
     * @return The orientation at time t
     */
    Angle getPosition(Duration t) const override;

    /**
     * Get angular velocity at time t
     *
     * @param t Duration elapsed since start of trajectory
     * @return The angular velocity at time t
     */
    AngularVelocity getVelocity(Duration t) const override;

    /**
     * Get angular acceleration at time t
     *
     * @param t Duration elapsed since start of trajectory
     * @return The angular acceleration at time t
     */
    AngularAcceleration getAcceleration(Duration t) const override;

    /**
     * Get total runtime of trajectory
     *
     * @return total time for trajectory
     */
    Duration getTotalTime() const override;

    /**
     * Get the trajectory parts that make up the generated trajectory. Note that all
     * values are in radians.
     *
     * @return Trajectory parts
     */
    const std::vector<BangBangTrajectory1D::TrajectoryPart> &getTrajectoryParts() const;

    private:
    BangBangTrajectory1D trajectory;
};

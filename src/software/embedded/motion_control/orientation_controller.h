#pragma once

#include "software/ai/navigator/trajectory/bang_bang_trajectory_1d_angular.h"
#include "software/embedded/motion_control/controller.h"
#include "software/embedded/motion_control/pid_controller.h"
#include "software/geom/angle.h"
#include "software/geom/angular_velocity.h"
#include "software/time/duration.h"

class OrientationController
    : public MotionController<Angle, BangBangTrajectory1DAngular, AngularVelocity>
{
   public:
    /**
     * Constructs an orientation controller that uses measurements over multiple
     * time intervals to calculate the target angular velocity to minimize error.
     */
    OrientationController() = default;

    /**
     * Given an orientation and target orientation, returns a target angular
     * velocity to minimize the error between the two.
     *
     * @param orientation The actual orientation.
     * @param target_trajectory The target angular trajectory.
     * @param elapsed_time The elapsed time since the trajectory was created.
     * @param delta_time The time passed since last time step.
     */
    AngularVelocity step(const Angle& orientation,
                         const BangBangTrajectory1DAngular& target_trajectory,
                         Duration elapsed_time, Duration delta_time) override;

    /**
     * Resets the state of this orientation controller.
     */
    void reset() override;

   private:
    // TODO(#3737): tune constants
    PidController w_pid_{0.4, 0.0, 0.0, 0.0};

    static constexpr double ANGULAR_DESTINATION_THRESHOLD_DEGREES = 5;
};

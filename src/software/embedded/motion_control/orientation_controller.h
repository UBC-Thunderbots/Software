#pragma once

#include "software/ai/navigator/trajectory/bang_bang_trajectory_1d_angular.h"
#include "software/embedded/motion_control/pid_controller.h"
#include "software/geom/angle.h"
#include "software/geom/angular_velocity.h"
#include "software/time/duration.h"

class OrientationController
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
     * @param target_orientation The target trajectory path.
     * @param delta_time The time passed since last time step.
     */
    AngularVelocity step(Angle orientation,
                         const BangBangTrajectory1DAngular& angular_trajectory,
                         Duration time_since_trajectory_creation,
                         double delta_time = 1.0);
    /**
     * Resets the state of this orientation controller.
     */
    void reset();

   private:
    // TODO: tune constants
    PidController w_pid_{0.7, 0.0, 2.0, 0.0};
    PidController w_pid_close_{2.0, 0.0, 4.0, 0.0};

    static constexpr double ANGULAR_PURE_PID_THRESHOLD_DEGREES = 25.0;
};

#pragma once

#include "software/ai/navigator/trajectory/bang_bang_trajectory_1d_angular.h"
#include "software/embedded/motion_control/controller.h"
#include "software/embedded/motion_control/pid_controller.h"
#include "software/geom/angle.h"
#include "software/geom/angular_velocity.h"
#include "software/time/duration.h"

// TODO(#3737): tune constants
// PID gains for OrientationController's underlying heading PID controller. Defaults
// preserve the previously hardcoded tuning; robot_config.toml can override these on a
// per-robot basis.
struct OrientationControllerConfig
{
    double kp           = 0.4;
    double ki           = 0.0;
    double kd           = 0.0;
    double max_integral = 0.0;
};

class OrientationController
    : public MotionController<Angle, BangBangTrajectory1DAngular, AngularVelocity>
{
   public:
    /**
     * Constructs an orientation controller that uses measurements over multiple
     * time intervals to calculate the target angular velocity to minimize error.
     *
     * @param config The PID gains to use for the underlying heading PID controller.
     */
    explicit OrientationController(
        const OrientationControllerConfig& config = OrientationControllerConfig());

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
    PidController<double> w_pid_;

    static constexpr double ANGULAR_DESTINATION_THRESHOLD_DEGREES = 5;
};

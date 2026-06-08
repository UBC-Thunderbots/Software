#include "software/embedded/motion_control/orientation_controller.h"

AngularVelocity OrientationController::step(
    const Angle& orientation, const BangBangTrajectory1DAngular& target_trajectory,
    Duration elapsed_time, Duration delta_time)
{
    const Angle difference_from_target =
        (target_trajectory.getDestination() - orientation).clamp();

    if (difference_from_target.abs().toDegrees() < ANGULAR_PURE_PID_THRESHOLD_DEGREES ||
        target_trajectory.getTotalTime() - elapsed_time.toSeconds() <
            PURE_PID_THRESHOLD_TIME)
    {
        // if target orientation is close enough, use pure PID for angular velocity
        return AngularVelocity::fromRadians(w_pid_close_.step(
            difference_from_target.toRadians(), delta_time.toSeconds()));
    }
    else
    {
        // feedforward trajectory angular velocity with small pid control effort
        const Angle error_angular =
            (target_trajectory.getPosition(elapsed_time.toSeconds()) - orientation)
                .clamp();
        const AngularVelocity pid_effort_angular = AngularVelocity::fromRadians(
            w_pid_.step(error_angular.toRadians(), delta_time.toSeconds()));
        return target_trajectory.getVelocity(elapsed_time.toSeconds()) +
               pid_effort_angular;
    }
}

void OrientationController::reset()
{
    w_pid_.reset();
    w_pid_close_.reset();
}

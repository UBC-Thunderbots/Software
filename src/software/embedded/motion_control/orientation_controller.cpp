#include "software/embedded/motion_control/orientation_controller.h"


AngularVelocity OrientationController::step(
    Angle orientation, const BangBangTrajectory1DAngular& angular_trajectory,
    Duration time_since_trajectory_creation, double delta_time)
{
    const Angle difference_from_target =
        (angular_trajectory.getDestination() - orientation).clamp();

    if (difference_from_target.abs().toDegrees() < ANGULAR_PURE_PID_THRESHOLD_DEGREES)
    {
        // if target orientation is close enough, use pure PID for angular velocity
        return AngularVelocity::fromRadians(
            w_pid_close_.step(difference_from_target.toRadians(), delta_time));
    }
    else
    {
        // feedforward trajectory angular velocity with small pid control effort
        const Angle error_angular =
            (angular_trajectory.getPosition(time_since_trajectory_creation.toSeconds()) -
             orientation)
                .clamp();
        const AngularVelocity pid_effort_angular = AngularVelocity::fromRadians(
            w_pid_.step(error_angular.toRadians(), delta_time));
        return angular_trajectory.getVelocity(
                   time_since_trajectory_creation.toSeconds()) +
               pid_effort_angular;
    }
}

void OrientationController::reset()
{
    w_pid_.reset();
    w_pid_close_.reset();
}

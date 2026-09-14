#include "software/embedded/motion_control/orientation_controller.h"

AngularVelocity OrientationController::step(
    const Angle& orientation, const BangBangTrajectory1DAngular& target_trajectory,
    Duration elapsed_time, Duration delta_time)
{
    // feedforward trajectory angular velocity with small pid control effort
    const Angle error_angular =
        (target_trajectory.getPosition(elapsed_time.toSeconds()) - orientation).clamp();
    const AngularVelocity pid_effort_angular = AngularVelocity::fromRadians(
        w_pid_.step(error_angular.toRadians(), delta_time.toSeconds()));

    AngularVelocity angular_velocity =
        target_trajectory.getVelocity(elapsed_time.toSeconds()) + pid_effort_angular;

    const Angle orientation_to_destination =
        orientation.minDiff(target_trajectory.getDestination());
    // Dampen angular velocity as we get closer to the destination to reduce jittering
    if (orientation_to_destination.toDegrees() < ANGULAR_DESTINATION_THRESHOLD_DEGREES)
    {
        angular_velocity *= orientation_to_destination.toDegrees() /
                            ANGULAR_DESTINATION_THRESHOLD_DEGREES;
    }
    return angular_velocity;
}

void OrientationController::reset()
{
    w_pid_.reset();
}

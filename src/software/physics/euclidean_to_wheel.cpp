#include "euclidean_to_wheel.h"

#include <cmath>

#include "shared/2021_robot_constants.h"
#include "software/logger/logger.h"

EuclideanToWheel::EuclideanToWheel(const RobotConstants_t &robot_constants)
{
    // import robot constants
    front_wheel_angle_phi_rad_  = robot_constants.front_wheel_angle_deg * M_PI / 180.;
    rear_wheel_angle_theta_rad_ = robot_constants.back_wheel_angle_deg * M_PI / 180.;

    // clang-format off
    euclidean_to_wheel_velocity_D <<
        -sin(front_wheel_angle_phi_rad_), cos(front_wheel_angle_phi_rad_), 1,
        -sin(front_wheel_angle_phi_rad_), -cos(front_wheel_angle_phi_rad_), 1,
        sin(rear_wheel_angle_theta_rad_), -cos(rear_wheel_angle_theta_rad_), 1,
        sin(rear_wheel_angle_theta_rad_), cos(rear_wheel_angle_theta_rad_), 1;
    // clang-format on
}

WheelSpace_t EuclideanToWheel::getTargetWheelSpeeds(
    const EuclideanSpace_t &target_euclidean_velocity)
{
    return euclidean_to_wheel_velocity_D * target_euclidean_velocity;
}


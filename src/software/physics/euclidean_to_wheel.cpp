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

    auto i =
        1 / (2 * sin(front_wheel_angle_phi_rad_) + 2 * sin(rear_wheel_angle_theta_rad_));
    auto j =
        cos(rear_wheel_angle_theta_rad_) / (2 * pow(cos(front_wheel_angle_phi_rad_), 2) +
                                            2 * pow(cos(rear_wheel_angle_theta_rad_), 2));
    auto k = sin(rear_wheel_angle_theta_rad_) /
             (2 * sin(front_wheel_angle_phi_rad_) + 2 * sin(rear_wheel_angle_theta_rad_));

    //clang-format off
    wheel_to_euclidean_velocity_D_inverse << -i, -i, i, i, j, -j, -(1 - j), (1 - j), k, k,
        (1 - k), (1 - k);
    //clang-format on
}

WheelSpace_t EuclideanToWheel::getWheelVelocity(
    const EuclideanSpace_t &euclidean_velocity)
{
    return euclidean_to_wheel_velocity_D * euclidean_velocity;
}

EuclideanSpace_t EuclideanToWheel::getEuclideanVelocity(
    const WheelSpace_t &wheel_velocity)
{
    return wheel_to_euclidean_velocity_D_inverse * wheel_velocity;
}

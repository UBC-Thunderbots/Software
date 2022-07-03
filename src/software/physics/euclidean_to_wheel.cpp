#include "euclidean_to_wheel.h"

#include <cmath>

#include "shared/2021_robot_constants.h"
#include "software/logger/logger.h"

EuclideanToWheel::EuclideanToWheel(const RobotConstants_t &robot_constants)
{
    // import robot constants
    front_wheel_angle_phi_rad_  = robot_constants.front_wheel_angle_deg * M_PI / 180.;
    rear_wheel_angle_theta_rad_ = robot_constants.back_wheel_angle_deg * M_PI / 180.;
    wheel_radius_m_ = robot_constants.wheel_radius_meters;

    // clang-format off
    euclidean_to_wheel_velocity_D_ <<
        -sin(front_wheel_angle_phi_rad_), cos(front_wheel_angle_phi_rad_), 1,
        -sin(front_wheel_angle_phi_rad_), -cos(front_wheel_angle_phi_rad_), 1,
        sin(rear_wheel_angle_theta_rad_), -cos(rear_wheel_angle_theta_rad_), 1,
        sin(rear_wheel_angle_theta_rad_), cos(rear_wheel_angle_theta_rad_), 1;
    // clang-format on
    
    auto i = 1 / (2*sin(front_wheel_angle_phi_rad_) + 2*sin(rear_wheel_angle_theta_rad_));
    auto j = cos(rear_wheel_angle_theta_rad_) / (2*pow(cos(front_wheel_angle_phi_rad_), 2) + 2*pow(cos(rear_wheel_angle_theta_rad_), 2));
    auto k = sin(rear_wheel_angle_theta_rad_) / (2*sin(front_wheel_angle_phi_rad_) + 2*sin(rear_wheel_angle_theta_rad_));
    
    //clang-format off
    wheel_to_euclidean_velocity_D_inverse_ <<
    -i, -i, i, i,
    j, -j, -(1 - j), (1 - j),
    k, k, (1 - k), (1 - k);
    //clang-format on
}

WheelSpace_t EuclideanToWheel::getWheelVelocity(
    EuclideanSpace_t euclidean_velocity)
{
    // need to multiply the angular velocity by the wheel radius
    // ref: http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf pg 8
    euclidean_velocity[2] = euclidean_velocity[2] * wheel_radius_m_;

    return euclidean_to_wheel_velocity_D_ * euclidean_velocity;
}

EuclideanSpace_t EuclideanToWheel::getEuclideanVelocity(
    const WheelSpace_t &wheel_velocity)
{
    EuclideanSpace_t euclidean_velocity = wheel_to_euclidean_velocity_D_inverse_ * wheel_velocity;
 
    // need to divide the angular velocity by the wheel radius
    // ref: http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf pg 8
    euclidean_velocity[2] = euclidean_velocity[2] / wheel_radius_m_;

    return euclidean_velocity;
}

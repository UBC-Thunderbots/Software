#include "euclidean_to_wheel.h"

#include <cmath>

#include "shared/2021_robot_constants.h"

EuclideanToWheel::EuclideanToWheel(const RobotConstants_t &robot_constants)
    : robot_radius_m_(robot_constants.robot_radius_m)
{
    // Phi, the angle between the hemisphere line of the robot and the front wheel axles
    // [rads]
    auto p = robot_constants.front_wheel_angle_deg * M_PI / 180.;

    // Theta, the angle between the hemisphere line of the robot and the rear wheel axles
    // [rads]
    auto t = robot_constants.back_wheel_angle_deg * M_PI / 180.;

    // Caching repeated calculations
    auto cos_p = std::cos(p);
    auto cos_t = std::cos(t);
    auto sin_p = std::sin(p);
    auto sin_t = std::sin(t);

    // clang-format off
    euclidean_to_wheel_velocity_D_ <<
        -sin_p,  cos_p, 1,
        -sin_p, -cos_p, 1,
         sin_t, -cos_t, 1,
         sin_t,  cos_t, 1;
    // clang-format on

    // Inverse of euclidean to wheel matrix (D) for converting wheel velocity to euclidean
    // velocity.
    // NOTE: The pseudoinverse given in the paper
    // (http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf pg 17) is incorrect. The inverse
    // was calculated using Wolfram Alpha: https://bit.ly/3A08BJX
    auto i = 1 / (2 * sin_p + 2 * sin_t);

    auto j_denom = 2 * cos_p * cos_p + 2 * cos_t * cos_t;
    auto j1      = cos_p / j_denom;
    auto j2      = cos_t / j_denom;

    auto k_denom = 2 * sin_p + 2 * sin_t;
    auto k1      = sin_t / k_denom;
    auto k2      = sin_p / k_denom;

    // clang-format off
    wheel_to_euclidean_velocity_D_inverse_ <<
        -i,  -i,   i,  i,
        j1, -j1, -j2, j2,
        k1,  k1,  k2, k2;
    // clang-format on
}

WheelSpace_t EuclideanToWheel::getWheelVelocity(EuclideanSpace_t euclidean_velocity) const
{
    // need to multiply the angular velocity by the robot radius to
    // calculate the wheel velocity (robot tangential velocity)
    // ref: http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf pg 8
    euclidean_velocity[2] = euclidean_velocity[2] * robot_radius_m_;

    return euclidean_to_wheel_velocity_D_ * euclidean_velocity;
}

EuclideanSpace_t EuclideanToWheel::getEuclideanVelocity(
    const WheelSpace_t &wheel_velocity) const
{
    EuclideanSpace_t euclidean_velocity =
        wheel_to_euclidean_velocity_D_inverse_ * wheel_velocity;

    // The above multiplication will calculate the tangential robot
    // velocity. This can be divided by the robot radius to calculate
    // the angular velocity
    // ref: http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf pg 8
    euclidean_velocity[2] = euclidean_velocity[2] / robot_radius_m_;

    return euclidean_velocity;
}

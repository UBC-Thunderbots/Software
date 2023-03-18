#include "euclidean_to_three_wheel.h"

#include <cmath>

#include "shared/2021_robot_constants.h"

EuclideanToThreeWheel::EuclideanToThreeWheel(const RobotConstants_t &robot_constants)
    : robot_radius_m_(robot_constants.robot_radius_m)
{
    // Phi, Assuming that all three wheels are evenly spaced around the robot
    double p = 60. * M_PI / 180.;

    double cos_p = std::cos(p);
    double sin_p = std::sin(p);
    double r = robot_radius_m_;

    // clang-format off
    euclidean_to_wheel_velocity_D_ <<
             -1,      0, r,
        cos_p, -sin_p, r,
         cos_p, sin_p, r;
    // clang-format on

    // Inverse of euclidean to wheel matrix (D) for converting wheel velocity to euclidean
    // velocity.
    // Calculated using Wolfram Alpha: https://www.wolframalpha.com/input?i=inverse+of+%5B%5B0%2C+1%2C+r%5D%2C%5B-sin%28phi%29%2C+-cos%28phi%29%2C+r%5D%2C%5Bsin%28phi%29%2C+-cos%28phi%29%2C+r%5D%5D
    double i = 1 / (2 * sin_p);

    double j1 = 1 / (cos_p + 1);
    double j2 = -1 / (2 * cos_p + 2);

    double k1 = cos_p / (r * cos_p + r);
    double k2 = 1 / (2 * r * cos_p + 2 * r);

    // clang-format off
    wheel_to_euclidean_velocity_D_inverse_ <<
         0, -i,  i,
        j1, j2, j2,
        k1, k2, k2;
    // clang-format on
}

ThreeWheelSpace_t EuclideanToThreeWheel::getWheelVelocity(const EuclideanSpace_t& euclidean_velocity) const
{
    return euclidean_to_wheel_velocity_D_ * euclidean_velocity;
}

EuclideanSpace_t EuclideanToThreeWheel::getEuclideanVelocity(
    const ThreeWheelSpace_t &wheel_velocity) const
{
    return wheel_to_euclidean_velocity_D_inverse_ * wheel_velocity;
}

#include "euclidean_to_wheel.h"

#include <cmath>

#include "proto/message_translation/tbots_geometry.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "shared/robot_constants.h"
#include "software/geom/angular_velocity.h"
#include "software/geom/vector.h"

// get rid of this stupid ifdef, and then look at the issue 

// Turn this to 
#if CHECK_VERSION(2026)
EuclideanToWheel::EuclideanToWheel(const robot_constants::RobotConstants& robot_constants)
    : robot_constants_(robot_constants)
{
    // Angles [rads]
    auto p = robot_constants_.front_wheel_angle_deg * M_PI / 180.;
    auto t = robot_constants_.back_wheel_angle_deg * M_PI / 180.;

    // Beta is angle of wheel rolling direction relative to X-axis
    // LOOK AT (software)/src/shared/robot_constants.h to see X-axis
    double b_fr = -(M_PI - p);
    double b_fl = -p;
    double b_bl = t;
    double b_br = (M_PI - t);

    // Mapped to the robot frame: +X = Forward, +Y = Left
    double fr_x =  0.03485, fr_y = -0.06632;
    double fl_x =  0.03485, fl_y =  0.06632;
    double bl_x = -0.04985, bl_y =  0.05592;
    double br_x = -0.04985, br_y = -0.05592;


    // Assuming that CCW when looking end of shaft into motor is the positive direction.
    // Formula is u_1 = v_x * cos(B_1) + v_y * sin(B_1) + W * (x_1 * sin(B_1) - y_1 * cos(B_1))

    // clang-format off
    euclidean_to_wheel_velocity_D_ <<
         std::cos(b_fr), std::sin(b_fr), (fr_x * std::sin(b_fr) - fr_y * std::cos(b_fr)),
         std::cos(b_fl), std::sin(b_fl), (fl_x * std::sin(b_fl) - fl_y * std::cos(b_fl)),
         std::cos(b_bl), std::sin(b_bl), (bl_x * std::sin(b_bl) - bl_y * std::cos(b_bl)),
         std::cos(b_br), std::sin(b_br), (br_x * std::sin(b_br) - br_y * std::cos(b_br));
    // clang-format on

    // Calculate Pseudo-inverse dynamically
    wheel_to_euclidean_velocity_D_inverse_ =
        (euclidean_to_wheel_velocity_D_.transpose() * euclidean_to_wheel_velocity_D_)
            .inverse() *
        euclidean_to_wheel_velocity_D_.transpose();
}

#else

EuclideanToWheel::EuclideanToWheel(const robot_constants::RobotConstants& robot_constants)
    : robot_constants_(robot_constants)
{
    // Phi, the angle between the hemisphere line of the robot and the front wheel axles
    // [rads]
    auto p = robot_constants_.front_wheel_angle_deg * M_PI / 180.;

    // Theta, the angle between the hemisphere line of the robot and the rear wheel axles
    // [rads]
    auto t = robot_constants_.back_wheel_angle_deg * M_PI / 180.;

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

#endif

// Why was this DEBUG_WHEEL
#if CHECK_VERSION(2026)
WheelSpace_t EuclideanToWheel::getWheelVelocity(EuclideanSpace_t euclidean_velocity) const
{
    // The generalized D matrix natively handles the w -> tangential velocity
    // conversion because its third column already represents the lever arm in meters.
    return euclidean_to_wheel_velocity_D_ * euclidean_velocity;
}


EuclideanSpace_t EuclideanToWheel::getEuclideanVelocity(
    const WheelSpace_t& wheel_velocity) const
{
    // The D inverse matrix natively outputs w in rad/s.
    return wheel_to_euclidean_velocity_D_inverse_ * wheel_velocity;
}

#else

WheelSpace_t EuclideanToWheel::getWheelVelocity(EuclideanSpace_t euclidean_velocity) const
{
    // need to multiply the angular velocity by the robot radius to
    // calculate the wheel velocity (robot tangential velocity)
    // ref: http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf pg 8
    euclidean_velocity[2] = euclidean_velocity[2] * robot_constants_.robot_radius_m;

    return euclidean_to_wheel_velocity_D_ * euclidean_velocity;
}

EuclideanSpace_t EuclideanToWheel::getEuclideanVelocity(
    const WheelSpace_t& wheel_velocity) const
{
    EuclideanSpace_t euclidean_velocity =
        wheel_to_euclidean_velocity_D_inverse_ * wheel_velocity;

    // The above multiplication will calculate the tangential robot
    // velocity. This can be divided by the robot radius to calculate
    // the angular velocity
    // ref: http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf pg 8
    euclidean_velocity[2] = euclidean_velocity[2] / robot_constants_.robot_radius_m;

    return euclidean_velocity;
}
#endif

WheelSpace_t EuclideanToWheel::rampWheelVelocity(
    const WheelSpace_t& current_wheel_velocity, const WheelSpace_t& target_wheel_velocity,
    const double& time_to_ramp)
{
    double allowed_acceleration =
        static_cast<double>(robot_constants_.motor_max_acceleration_m_per_s_2);
    double max_allowable_wheel_velocity =
        static_cast<double>(robot_constants_.robot_max_speed_m_per_s);

    // ramp wheel velocity
    WheelSpace_t ramp_wheel_velocity;

    // calculate max allowable wheel velocity delta using dv = a*t
    auto allowable_delta_wheel_velocity = allowed_acceleration * time_to_ramp;

    // Ramp wheel velocity vector
    // Step 1: Find absolute max velocity delta
    auto delta_target_wheel_velocity = target_wheel_velocity - current_wheel_velocity;
    auto max_delta_target_wheel_velocity =
        delta_target_wheel_velocity.cwiseAbs().maxCoeff();

    // Step 2: Compare max delta velocity against the calculated maximum
    if (max_delta_target_wheel_velocity > allowable_delta_wheel_velocity)
    {
        // Step 3: If larger, scale down to allowable max
        ramp_wheel_velocity =
            (delta_target_wheel_velocity / max_delta_target_wheel_velocity) *
                allowable_delta_wheel_velocity +
            current_wheel_velocity;
    }
    else
    {
        // If smaller, go straight to target
        ramp_wheel_velocity = target_wheel_velocity;
    }

    // find absolute max wheel velocity
    auto max_ramp_wheel_velocity = ramp_wheel_velocity.cwiseAbs().maxCoeff();

    // compare against max wheel velocity
    if (max_ramp_wheel_velocity > max_allowable_wheel_velocity)
    {
        // if larger, scale down to max
        ramp_wheel_velocity = (ramp_wheel_velocity / max_ramp_wheel_velocity) *
                              max_allowable_wheel_velocity;
    }

    return ramp_wheel_velocity;
}

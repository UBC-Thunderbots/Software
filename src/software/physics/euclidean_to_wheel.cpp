#include "euclidean_to_wheel.h"

#include <cmath>

#include "proto/message_translation/tbots_geometry.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "shared/robot_constants.h"
#include "software/geom/angular_velocity.h"
#include "software/geom/vector.h"

EuclideanToWheel::EuclideanToWheel(const robot_constants::RobotConstants& robot_constants)
    : robot_constants_(robot_constants)
{
    // Angles [rads]
    const double p = robot_constants_.front_wheel_angle_deg * M_PI / 180.;
    const double t = robot_constants_.back_wheel_angle_deg * M_PI / 180.;

    // Beta is angle of wheel rolling direction relative to X-axis
    // LOOK AT (software)/src/shared/robot_constants.h to see X-axis
    const double b_fr = -(M_PI - p);
    const double b_fl = -p;
    const double b_bl = t;
    const double b_br = (M_PI - t);

    // Mapped to the robot frame: +X = Forward, +Y = Left
    const double fr_x = robot_constants_.fr_x_pos_meters;
    const double fr_y = robot_constants_.fr_y_pos_meters;
    const double fl_x = robot_constants_.fl_x_pos_meters;
    const double fl_y = robot_constants_.fl_y_pos_meters;
    const double bl_x = robot_constants_.bl_x_pos_meters;
    const double bl_y = robot_constants_.bl_y_pos_meters;
    const double br_x = robot_constants_.br_x_pos_meters;
    const double br_y = robot_constants_.br_y_pos_meters;


    // Assuming that CCW when looking end of shaft into motor is the positive direction.
    // Formula is u_1 = v_x * cos(B_1) + v_y * sin(B_1) + W * (x_1 * sin(B_1) - y_1 *
    // cos(B_1))

    // clang-format off
    euclidean_to_wheel_velocity_D_ <<
         -std::cos(b_fr), -std::sin(b_fr), -(fr_x * std::sin(b_fr) - fr_y * std::cos(b_fr)),
         -std::cos(b_fl), -std::sin(b_fl), -(fl_x * std::sin(b_fl) - fl_y * std::cos(b_fl)),
         -std::cos(b_bl), -std::sin(b_bl), -(bl_x * std::sin(b_bl) - bl_y * std::cos(b_bl)),
         -std::cos(b_br), -std::sin(b_br), -(br_x * std::sin(b_br) - br_y * std::cos(b_br));
    // clang-format on

    // Inverse of euclidean to wheel matrix (D) for converting wheel velocity to euclidean
    // velocity.
    // NOTE: The pseudoinverse given in the paper
    // (http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf pg 17) is incorrect. The inverse
    // was calculated using Wolfram Alpha: https://bit.ly/3A08BJX

    // Calculate Pseudo-inverse dynamically

    wheel_to_euclidean_velocity_D_inverse_ =
        euclidean_to_wheel_velocity_D_.completeOrthogonalDecomposition().pseudoInverse();
}

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

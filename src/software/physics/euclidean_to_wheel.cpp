#include "euclidean_to_wheel.h"

#include <cmath>

#include "shared/2021_robot_constants.h"
#include "software/geom/vector.h"
#include "software/geom/angular_velocity.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/message_translation/tbots_geometry.h"

EuclideanToWheel::EuclideanToWheel(const RobotConstants_t &robot_constants)
    : robot_radius_m_(robot_constants.robot_radius_m),
      robot_constants(robot_constants)
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

WheelSpace_t EuclideanToWheel::rampWheelVelocity(
        const WheelSpace_t& current_wheel_velocity,
        const EuclideanSpace_t& target_euclidean_velocity,
        const double& time_to_ramp)
{
    double allowed_acceleration = static_cast<double>(robot_constants.robot_max_acceleration_m_per_s_2);
    double max_allowable_wheel_velocity = static_cast<double>(robot_constants.robot_max_acceleration_m_per_s_2);

    // ramp wheel velocity
    WheelSpace_t ramp_wheel_velocity;

    // calculate max allowable wheel velocity delta using dv = a*t
    auto allowable_delta_wheel_velocity = allowed_acceleration * time_to_ramp;

    // convert euclidean to wheel velocity
    WheelSpace_t target_wheel_velocity =
            getWheelVelocity(target_euclidean_velocity);

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

std::unique_ptr<TbotsProto::DirectControlPrimitive> EuclideanToWheel::rampWheelVelocity(
        const std::pair<Vector, AngularVelocity> current_velocity,
        const std::unique_ptr<TbotsProto::DirectControlPrimitive> target_velocity_primitive,
        const double& time_to_ramp)
{
    EuclideanSpace_t target_euclidean_velocity = {};

    TbotsProto::MotorControl motor_control = target_velocity_primitive->motor_control();
    if (motor_control.has_direct_per_wheel_control())
    {
        TbotsProto::MotorControl_DirectPerWheelControl direct_per_wheel = motor_control.direct_per_wheel_control();
        WheelSpace_t wheel_velocity = {
                direct_per_wheel.front_left_wheel_velocity(),
                direct_per_wheel.front_left_wheel_velocity(),
                direct_per_wheel.front_left_wheel_velocity(),
                direct_per_wheel.front_left_wheel_velocity(),
        };
        target_euclidean_velocity = getEuclideanVelocity(wheel_velocity);
    }
    else if (target_velocity_primitive->motor_control().has_direct_velocity_control())
    {
        TbotsProto::MotorControl_DirectVelocityControl direct_velocity = motor_control.direct_velocity_control();
        target_euclidean_velocity = {
                -(int)direct_velocity.velocity().y_component_meters(),
                (int)direct_velocity.velocity().x_component_meters(),
                direct_velocity.angular_velocity().radians_per_second()
        };
    }

    EuclideanSpace_t current_euclidean_velocity = {
            -current_velocity.first.y(),
            current_velocity.first.x(),
            current_velocity.second.toRadians()
    };

    WheelSpace_t current_four_wheel_velocity = getWheelVelocity(current_euclidean_velocity);

    WheelSpace_t ramped_four_wheel = rampWheelVelocity(
            current_four_wheel_velocity,
            target_euclidean_velocity,
            time_to_ramp);

    EuclideanSpace_t ramped_euclidean = getEuclideanVelocity(ramped_four_wheel);

    auto mutable_direct_velocity = target_velocity_primitive->mutable_motor_control()->mutable_direct_velocity_control();
    *(mutable_direct_velocity->mutable_velocity()) = *createVectorProto({ramped_euclidean[1], -ramped_euclidean[0]});
    *(mutable_direct_velocity->mutable_angular_velocity()) = *createAngularVelocityProto(
            AngularVelocity::fromRadians(ramped_euclidean[2]));

    return std::move(target_velocity_primitive);

}

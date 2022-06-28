#include "euclidean_to_wheel.h"

#include <cmath>

#include "shared/2021_robot_constants.h"

EuclideanToWheel::EuclideanToWheel(const int &control_loop_frequency_Hz,
                                   const RobotConstants_t &robot_constants)
{
    // Set to period of control loop.
    delta_t_s_ = 1.0f / static_cast<float>(control_loop_frequency_Hz);

    // import robot constants
    robot_mass_M_kg_           = robot_constants.mass_kg;
    robot_radius_R_m_          = robot_constants.robot_radius_m;
    inertial_factor_alpha_m_   = robot_constants.inertial_factor;
    front_wheel_angle_phi_rad_ = robot_constants.front_wheel_angle_deg * M_PI / 180.;
    rear_wheel_angle_theta_rad_ =
        (robot_constants.back_wheel_angle_deg - 90.) * M_PI / 180.;

    // calculate DC_alpha matrix
    // ref: http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf pg 17
    auto a =
        pow(sin(front_wheel_angle_phi_rad_), 2) - pow(cos(front_wheel_angle_phi_rad_), 2);
    auto b = -sin(front_wheel_angle_phi_rad_) * sin(rear_wheel_angle_theta_rad_) -
             cos(front_wheel_angle_phi_rad_) * cos(rear_wheel_angle_theta_rad_);
    auto c = -sin(front_wheel_angle_phi_rad_) * sin(rear_wheel_angle_theta_rad_) +
             cos(front_wheel_angle_phi_rad_) * cos(rear_wheel_angle_theta_rad_);
    auto d = pow(sin(rear_wheel_angle_theta_rad_), 2) -
             pow(cos(rear_wheel_angle_theta_rad_), 2);
    Eigen::Matrix4d e;
    e << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
    Eigen::Matrix4d f;
    f << 1, a, b, c, a, 1, c, b, b, c, 1, d, c, b, d, 1;
    wheel_force_to_wheel_speed_delta_D_C_alpha_ =
        1 / (robot_mass_M_kg_ * inertial_factor_alpha_m_) * e + 1 / robot_mass_M_kg_ * f;

    // calculate D inverse matrix
    // ref: http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf pg 17
    auto i =
        1 / (2 * sin(front_wheel_angle_phi_rad_) + 2 * sin(rear_wheel_angle_theta_rad_));
    auto j =
        cos(front_wheel_angle_phi_rad_) / (2 * pow(cos(front_wheel_angle_phi_rad_), 2) +
                                           2 * pow(cos(rear_wheel_angle_theta_rad_), 2));
    auto k = sin(rear_wheel_angle_theta_rad_) /
             (2 * sin(front_wheel_angle_phi_rad_) + 2 * sin(rear_wheel_angle_theta_rad_));
    wheel_speed_to_euclidean_velocity_D_inverse_ << -i, -i, i, i, j, -j, -(1. - j),
        (1. - j), k, k, (1 - k), (1 - k);
}

/**
 * Step 1: Convert Euclidean velocity to acceleration.
 * Step 2: Calculate the target translational wheel forces.
 * Step 3: Calculate the target rotational wheel forces.
 * Step 4: Sum the wheel forces.
 * Step 5: Convert wheel forces to speeds.
 */
WheelSpace_t EuclideanToWheel::getTargetWheelSpeeds(
    const EuclideanSpace_t &target_euclidean_velocity,
    const WheelSpace_t &current_wheel_speeds)
{
    // calculate current Euclidean velocity
    auto current_euclidean_velocity = getEuclideanVelocity(current_wheel_speeds);

    // calculate target Euclidean acceleration
    auto target_euclidean_acceleration =
        getEuclideanAcceleration(current_euclidean_velocity, target_euclidean_velocity);

    // calculate translational wheel forces
    auto target_translational_wheel_forces =
        getTranslationalWheelForces(target_euclidean_acceleration);

    // calculate rotational wheel forces
    auto target_rotational_wheel_forces =
        getRotationalWheelForces(target_euclidean_acceleration);

    // calculate delta wheel speed from the sum of the target forces
    auto delta_wheel_speeds = getWheelSpeedsDelta(target_translational_wheel_forces +
                                                  target_rotational_wheel_forces);

    // calculate final velocity given current + wheel speed delta
    return current_wheel_speeds + delta_wheel_speeds;
}

EuclideanSpace_t EuclideanToWheel::getEuclideanVelocity(const WheelSpace_t &wheel_speeds)
{
    return wheel_speed_to_euclidean_velocity_D_inverse_ * wheel_speeds;
}

EuclideanSpace_t EuclideanToWheel::getEuclideanAcceleration(
    const EuclideanSpace_t &initial_velocity,
    const EuclideanSpace_t &target_velocity) const
{
    return (target_velocity - initial_velocity) / delta_t_s_;
}

WheelSpace_t EuclideanToWheel::getTranslationalWheelForces(
    EuclideanSpace_t target_acceleration) const
{
    // x acceleration
    Eigen::Vector2d ax(target_acceleration(0) / sin(front_wheel_angle_phi_rad_),
                       target_acceleration(0) / sin(rear_wheel_angle_theta_rad_));

    Eigen::Matrix<double, 4, 2> ax_select;
    ax_select << -1, 0, -1, 0, 0, 1, 0, 1;

    // y acceleration
    Eigen::Vector2d ay(target_acceleration(1) / cos(front_wheel_angle_phi_rad_),
                       target_acceleration(1) / cos(rear_wheel_angle_theta_rad_));

    Eigen::Matrix<double, 4, 2> ay_select;
    ay_select << 1, 0, -1, 0, 0, -1, 0, 1;

    // sum x and y wheel forces
    return robot_mass_M_kg_ * (ax_select * ax + ay_select * ay);
}

WheelSpace_t EuclideanToWheel::getRotationalWheelForces(
    EuclideanSpace_t target_acceleration) const
{
    // calculate per wheel rotational force
    auto rotational_force = target_acceleration(2) / 4 * inertial_factor_alpha_m_ *
                            robot_mass_M_kg_ * robot_radius_R_m_;

    // apply force to each wheel
    WheelSpace_t target_force(rotational_force, rotational_force, rotational_force,
                              rotational_force);

    return target_force;
}

WheelSpace_t EuclideanToWheel::getWheelSpeedsDelta(
    const WheelSpace_t &target_wheel_forces)
{
    return delta_t_s_ *
           (wheel_force_to_wheel_speed_delta_D_C_alpha_ * target_wheel_forces);
}

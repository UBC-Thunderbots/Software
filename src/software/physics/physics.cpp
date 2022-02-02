#include "software/physics/physics.h"

#define _USE_MATH_DEFINES

#include <cmath>

#include "shared/2021_robot_constants.h"
#include "shared/constants.h"

Point calculateFuturePosition(const Point &initial_position,
                              const Vector &initial_velocity,
                              const Vector &constant_acceleration,
                              const Duration &duration_in_future)
{
    double seconds_in_future = duration_in_future.toSeconds();
    // Using second equation of motion from https://physics.info/motion-equations/
    // ∆s = v0*t + ½a*t^2
    double x1 = initial_position.x() + initial_velocity.x() * seconds_in_future +
                0.5 * constant_acceleration.x() * pow(seconds_in_future, 2);
    double y1 = initial_position.y() + initial_velocity.y() * seconds_in_future +
                0.5 * constant_acceleration.y() * pow(seconds_in_future, 2);

    return Point(x1, y1);
}

Vector calculateFutureVelocity(const Vector &initial_velocity,
                               const Vector &constant_acceleration,
                               const Duration &duration_in_future)
{
    double seconds_in_future = duration_in_future.toSeconds();
    double vx1 = initial_velocity.x() + seconds_in_future * constant_acceleration.x();
    double vy1 = initial_velocity.y() + seconds_in_future * constant_acceleration.y();
    return Vector(vx1, vy1);
}

EuclideanToFourWheel::EuclideanToFourWheel()
{
    auto robot_constants = create2021RobotConstants();

    // TODO: replace with Thunderloop polling rate constant
    // Set to period of control loop.
    delta_t_ = 1. / 200;

    // import robot constants
    robot_mass_M_            = robot_constants.mass_kg;
    robot_radius_R_          = robot_constants.robot_radius_m;
    inertial_factor_alpha_   = robot_constants.inertial_factor;
    front_wheel_angle_phi_   = robot_constants.front_wheel_angle_deg * M_PI / 180.;
    rear_wheel_angle_theta_  = (robot_constants.back_wheel_angle_deg - 90.) * M_PI / 180.;

    // calculate coupling matrices
    // ref: http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf
    auto a = pow(sin(front_wheel_angle_phi_), 2) - pow(cos(front_wheel_angle_phi_), 2);
    auto b = -sin(front_wheel_angle_phi_) * sin(rear_wheel_angle_theta_) -
             cos(front_wheel_angle_phi_) * cos(rear_wheel_angle_theta_);
    auto c = -sin(front_wheel_angle_phi_) * sin(rear_wheel_angle_theta_) +
             cos(front_wheel_angle_phi_) * cos(rear_wheel_angle_theta_);
    auto d = pow(sin(rear_wheel_angle_theta_), 2) - pow(cos(rear_wheel_angle_theta_), 2);
    Eigen::Matrix4d e;
    e << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
    Eigen::Matrix4d f;
    f << 1, a, b, c, a, 1, c, b, b, c, 1, d, c, b, d, 1;
    wheel_force_to_wheel_speed_delta_D_C_alpha_ =
        1 / (robot_mass_M_ * inertial_factor_alpha_) * e + 1 / robot_mass_M_ * f;

    auto i = 1 / (2 * sin(front_wheel_angle_phi_) + 2 * sin(rear_wheel_angle_theta_));
    auto j = cos(front_wheel_angle_phi_) / (2 * pow(cos(front_wheel_angle_phi_), 2) +
                                            2 * pow(cos(rear_wheel_angle_theta_), 2));
    auto k = sin(rear_wheel_angle_theta_) /
             (2 * sin(front_wheel_angle_phi_) + 2 * sin(rear_wheel_angle_theta_));
    wheel_speed_to_euclidean_velocity_D_inverse_ << -i, -i, i, i, j, -j, -(1. - j),
        (1. - j), k, k, (1 - k), (1 - k);
}

WheelSpace_t EuclideanToFourWheel::getTargetWheelSpeeds(
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

EuclideanSpace_t EuclideanToFourWheel::getEuclideanVelocity(
    const WheelSpace_t &wheel_speeds)
{
    return wheel_speed_to_euclidean_velocity_D_inverse_ * wheel_speeds;
}

EuclideanSpace_t EuclideanToFourWheel::getEuclideanAcceleration(
    const EuclideanSpace_t &initial_velocity,
    const EuclideanSpace_t &target_velocity) const
{
    return (target_velocity - initial_velocity) / delta_t_;
}

WheelSpace_t EuclideanToFourWheel::getTranslationalWheelForces(
    EuclideanSpace_t target_acceleration) const
{
    // x acceleration
    Eigen::Vector2d ax(target_acceleration(0) / sin(front_wheel_angle_phi_),
                       target_acceleration(0) / sin(rear_wheel_angle_theta_));

    Eigen::Matrix<double, 4, 2> ax_select;
    ax_select << -1, 0, -1, 0, 0, 1, 0, 1;

    // y acceleration
    Eigen::Vector2d ay(target_acceleration(1) / cos(front_wheel_angle_phi_),
                       target_acceleration(1) / cos(rear_wheel_angle_theta_));

    Eigen::Matrix<double, 4, 2> ay_select;
    ay_select << 1, 0, -1, 0, 0, -1, 0, 1;

    // sum x and y wheel forces
    return robot_mass_M_ * (ax_select * ax + ay_select * ay);
}

WheelSpace_t EuclideanToFourWheel::getRotationalWheelForces(
    EuclideanSpace_t target_acceleration) const
{
    // calculate per wheel rotational force
    auto rotational_force = target_acceleration(2) / 4 * inertial_factor_alpha_ *
                            robot_mass_M_ * robot_radius_R_;

    // apply force to each wheel
    WheelSpace_t target_force(rotational_force, rotational_force, rotational_force,
                              rotational_force);

    return target_force;
}

WheelSpace_t EuclideanToFourWheel::getWheelSpeedsDelta(
    const WheelSpace_t &target_wheel_forces)
{
    return delta_t_ * (wheel_force_to_wheel_speed_delta_D_C_alpha_ * target_wheel_forces);
}

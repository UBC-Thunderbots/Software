#pragma once

#include <Eigen/Dense>

#include "shared/robot_constants.h"

/**
 * Vector representation of 2D Euclidean space.
 *
 * 0: X
 * 1: Y
 * 2: W/Rotation
 */
typedef Eigen::Vector3d EuclideanSpace_t;

/**
 * Vector representation of the robot wheel space.
 *
 * 0: Front Right
 * 1: Front Left
 * 2: Back Left
 * 3: Back Right
 */
typedef Eigen::Vector4d WheelSpace_t;

class EuclideanToWheel
{
   public:
    EuclideanToWheel() = delete;

    /**
     * Initializes the Euclidean velocity to wheel speed conversion matrices.
     *
     * @param control_loop_frequency_Hz The frequency of the control loop.
     * @param robot_constants The constants of the robot we are computing for.
     */
    explicit EuclideanToWheel(const int &control_loop_frequency_Hz,
                              const RobotConstants_t &robot_constants);

    /**
     * Gets wheel velocity targets from the desired Euclidean velocity.
     *
     * @param target_euclidean_velocity The target Euclidean velocity.
     * @return The equivalent wheel speeds.
     */
    WheelSpace_t getTargetWheelSpeeds(const EuclideanSpace_t &target_euclidean_velocity,
                                      const WheelSpace_t &current_wheel_speeds);

   private:
    /**
     * The control loop time period [s].
     */
    double delta_t_s_{};

    /**
     * The mass of the robot [kg].
     */
    double robot_mass_M_kg_{};

    /**
     * The robot radius [m].
     */
    double robot_radius_R_m_{};

    /**
     * The mass distribution of the robot [m].
     */
    double inertial_factor_alpha_m_{};

    /**
     * The angle between the hemisphere line of the robot and the front wheel axles
     * [rads].
     */
    double front_wheel_angle_phi_rad_{};

    /**
     * The angle between the hemisphere line of the robot and the rear wheel axles [rads].
     */
    double rear_wheel_angle_theta_rad_{};

    /**
     * Wheel force to wheel speed delta [m/s].
     */
    Eigen::Matrix4d wheel_force_to_wheel_speed_delta_D_C_alpha_;

    /**
     * Wheel speed to Euclidean velocity coupling matrix.
     */
    Eigen::Matrix<double, 3, 4> wheel_speed_to_euclidean_velocity_D_inverse_;

    /**
     * Gets the equivalent Euclidean velocity from the measured wheel speeds.
     *
     * @param wheel_speeds The measured wheel speeds.
     * @return The equivalent robot Euclidean velocity.
     */
    EuclideanSpace_t getEuclideanVelocity(const WheelSpace_t &wheel_speeds);

    /**
     * Gets the target Euclidean acceleration.
     *
     * @param initial_velocity The initial velocity.
     * @param target_velocity The target velocity.
     * @return The Euclidean acceleration.
     */
    EuclideanSpace_t getEuclideanAcceleration(
        const EuclideanSpace_t &initial_velocity,
        const EuclideanSpace_t &target_velocity) const;

    /**
     * Gets the translational wheel forces.
     *
     * @refitem http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf pg 14
     * @param target_acceleration The target Euclidean acceleration.
     * @return The target translational wheel forces.
     */
    WheelSpace_t getTranslationalWheelForces(EuclideanSpace_t target_acceleration) const;

    /**
     * Gets the rotational wheel forces.
     *
     * @refitem http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf pg 14
     * @param target_acceleration The target Euclidean acceleration.
     * @return The target rotational wheel forces.
     */
    WheelSpace_t getRotationalWheelForces(EuclideanSpace_t target_acceleration) const;

    /**
     * Gets the wheel speed delta.
     *
     * @param target_wheel_forces The target wheel forces.
     * @return The target wheel speed delta.
     */
    WheelSpace_t getWheelSpeedsDelta(const WheelSpace_t &target_wheel_forces);
};

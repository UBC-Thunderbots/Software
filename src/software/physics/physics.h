#pragma once

#include <Eigen/Dense>

#include "software/geom/point.h"
#include "software/time/duration.h"

/**
 * Calculates the future position of a moving object
 *
 * @pre The units of the parameters are consistent, e.g. acceleration is in (units of
 * initial_position)/seconds^2
 *
 * @param initial_position The initial position of the object
 * @param initial_velocity The initial velocity of the object in position
 * @param constant_acceleration The constant acceleration applied to the object over time
 * @param duration_in_future Duration into the future
 *
 * @return future position of the object
 */
Point calculateFuturePosition(const Point &initial_position,
                              const Vector &intial_velocity,
                              const Vector &constant_acceleration,
                              const Duration &duration_in_future);

/**
 * Calculates the future velocity of a moving object
 *
 * @pre The units of the parameters are consistent, e.g. acceleration is in (units of
 * initial_velocity)/seconds
 *
 * @param initial_velocity The initial velocity of the object in position
 * @param constant_acceleration The constant acceleration applied to the object over time
 * @param duration_in_future Duration into the future
 *
 * @return future velocity of the object
 */
Vector calculateFutureVelocity(const Vector &intial_velocity,
                               const Vector &constant_acceleration,
                               const Duration &duration_in_future);


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

class EuclideanToFourWheel
{
   public:
    EuclideanToFourWheel() = delete;

    /**
     * Initializes the Euclidean velocity to wheel speed conversion matrices.
     *
     * @param front_wheel_angle_deg The angle between the hemisphere line and the front
     * wheel axles.
     * @param back_wheel_angle_deg The angle between the hemisphere line and the rear
     * wheel axles.
     */
    EuclideanToFourWheel(double front_wheel_angle_deg, double back_wheel_angle_deg);

    /**
     * Gets wheel velocity targets from the desired Euclidean velocity.
     *
     * Step 1: Convert Euclidean velocity to acceleration.
     * Step 2: Calculate the target translational wheel forces.
     * Step 3: Calculate the target rotational wheel forces.
     * Step 4: Sum the wheel forces.
     * Step 5: Convert wheel forces to speeds.
     *
     * @param target_euclidean_velocity The target Euclidean velocity.
     * @return The equivalent wheel speeds.
     */
    WheelSpace_t get_target_wheel_speeds(
        const EuclideanSpace_t &target_euclidean_velocity,
        const WheelSpace_t &current_wheel_speeds);

   private:
    /**
     * The control loop time period [s].
     */
    double delta_t_;

    /**
     * The mass of the robot [kg].
     */
    double robot_mass_M_;

    /**
     * The robot radius [m].
     */
    double robot_radius_R_;

    /**
     * The mass distribution of the robot [m].
     */
    double mass_distribution_alpha_;

    /**
     * The angle between the hemisphere line of the robot and the front wheel axles
     * [rads].
     */
    double front_wheel_angle_phi_;

    /**
     * The angle between the hemisphere line of the robot and the rear wheel axles [rads].
     */
    double rear_wheel_angle_theta_;

    /**
     * Wheel force to wheel speed delta [m/s].
     */
    Eigen::Matrix4d wheel_force_to_delta_wheel_speed_D_C_alpha_;

    /**
     * Euclidean acceleration to wheel force coupling matrix.
     */
    Eigen::Matrix<double, 3, 4> wheel_forces_to_euclidean_acceleration_C_alpha_;

    /**
     * Euclidean velocity to wheel speed coupling matrix.
     */
    Eigen::Matrix<double, 4, 3> euclidean_velocity_to_wheel_speed_D_;

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
    EuclideanSpace_t get_euclidean_velocity_(const WheelSpace_t &wheel_speeds);

    /**
     * Gets the target Euclidean acceleration.
     *
     * @param initial_velocity The initial velocity.
     * @param target_velocity The target velocity.
     * @return The Euclidean acceleration.
     */
    EuclideanSpace_t get_euclidean_acceleration_(
        const EuclideanSpace_t &initial_velocity,
        const EuclideanSpace_t &target_velocity) const;

    /**
     * Gets the translational wheel forces.
     *
     * @param target_acceleration The target Euclidean acceleration.
     * @return The target translational wheel forces.
     */
    WheelSpace_t get_translational_wheel_forces_(
        EuclideanSpace_t target_acceleration) const;

    /**
     * Gets the rotational wheel forces.
     *
     * @param target_acceleration The target Euclidean acceleration.
     * @return The target rotational wheel forces.
     */
    WheelSpace_t get_rotational_wheel_forces_(EuclideanSpace_t target_acceleration) const;

    /**
     * Gets the wheel speed delta.
     *
     * @param target_wheel_forces The target wheel forces.
     * @return The target wheel speed delta.
     */
    WheelSpace_t get_wheel_speeds_delta_(const WheelSpace_t &target_wheel_forces);
};

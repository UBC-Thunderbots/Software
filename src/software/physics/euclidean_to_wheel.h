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
     * @param robot_constants The constants of the robot we are computing for.
     */
    explicit EuclideanToWheel(const RobotConstants_t &robot_constants);

    /**
     * Gets the wheel velocity from the Euclidean velocity.
     *
     * @param euclidean_velocity The Euclidean velocity.
     * @return The equivalent wheel speeds.
     */
    WheelSpace_t getWheelVelocity(EuclideanSpace_t euclidean_velocity) const;

    /**
     * Gets the Euclidean velocity from the wheel velocity.
     *
     * @param wheel_velocity The wheel velocity.
     * @return The equivalent Euclidean velocity.
     */
    EuclideanSpace_t getEuclideanVelocity(const WheelSpace_t &wheel_velocity) const;

  private:
    /**
     * The radius of the robot in meters.
     */
    const double robot_radius_m_{};

    /**
     * Euclidean velocity to wheel velocity coupling matrix.
     *
     * ref: http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf pg 16
     */
    Eigen::Matrix<double, 4, 3> euclidean_to_wheel_velocity_D_;

    /**
     * Wheel velocity to Euclidean velocity coupling matrix.
     *
     * ref: http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf pg 16
     */
    Eigen::Matrix<double, 3, 4> wheel_to_euclidean_velocity_D_inverse_;
};

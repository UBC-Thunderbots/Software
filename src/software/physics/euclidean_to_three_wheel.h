#pragma once

#include <Eigen/Dense>

#include "shared/robot_constants.h"
#include "euclidean_to_wheel.h"

/**
 * Vector representation of the robot wheel space.
 *
 * 0: Front
 * 1: Back Left
 * 2: Back Right
 */
typedef Eigen::Vector3d ThreeWheelSpace_t;

// TODO: Update
static constexpr int FRONT_WHEEL_SPACE_INDEX = 0;
static constexpr int LEFT_WHEEL_SPACE_INDEX  = 1;
static constexpr int RIGHT_WHEEL_SPACE_INDEX = 2;

class EuclideanToThreeWheel
{
   public:
    EuclideanToThreeWheel() = delete;

    /**
     * Initializes the Euclidean velocity to wheel speed conversion matrices.
     *
     * @param robot_constants The constants of the robot we are computing for.
     */
    explicit EuclideanToThreeWheel(const RobotConstants_t &robot_constants);

    /**
     * Gets the wheel velocity from the Euclidean velocity.
     *
     * @param euclidean_velocity The Euclidean velocity.
     * @return The equivalent wheel speeds.
     */
    ThreeWheelSpace_t getWheelVelocity(const EuclideanSpace_t& euclidean_velocity) const;

    /**
     * Gets the Euclidean velocity from the wheel velocity.
     *
     * @param wheel_velocity The wheel velocity.
     * @return The equivalent Euclidean velocity.
     */
    EuclideanSpace_t getEuclideanVelocity(const ThreeWheelSpace_t &wheel_velocity) const;

   private:
    /**
     * The radius of the robot in meters.
     */
    const double robot_radius_m_{};

    /**
     * Euclidean velocity to wheel velocity coupling matrix.
     *
     * ref: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9316668&tag=1 pg 3
     */
    Eigen::Matrix<double, 3, 3> euclidean_to_wheel_velocity_D_;

    /**
     * Wheel velocity to Euclidean velocity coupling matrix.
     *
     * ref: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9316668&tag=1 pg 3
     */
    Eigen::Matrix<double, 3, 3> wheel_to_euclidean_velocity_D_inverse_;
};

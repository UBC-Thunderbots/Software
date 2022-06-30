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
     * Gets wheel velocity targets from the desired Euclidean velocity.
     *
     * @param target_euclidean_velocity The target Euclidean velocity.
     * @return The equivalent wheel speeds.
     */
    WheelSpace_t getTargetWheelSpeeds(const EuclideanSpace_t &target_euclidean_velocity);

   private:
    /**
     * The angle between the hemisphere line of the robot and the front wheel axles
     * [rads].
     */
    double front_wheel_angle_phi_rad_{};

    /**
     * The angle between the hemisphere line of the robot and the rear wheel axles [rads]
     */
    double rear_wheel_angle_theta_rad_{};

    /**
     * Euclidean velocity to wheel velocity coupling matrix.
     *
     * ref: http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf pg 16
     */
    Eigen::Matrix<double, 4, 3> euclidean_to_wheel_velocity_D;
};

#pragma once

#include <Eigen/Dense>

#include "proto/primitive.pb.h"
#include "shared/robot_constants.h"
#include "software/geom/vector.h"
#include "software/geom/angular_velocity.h"

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

static constexpr int FRONT_RIGHT_WHEEL_SPACE_INDEX = 0;
static constexpr int FRONT_LEFT_WHEEL_SPACE_INDEX  = 1;
static constexpr int BACK_LEFT_WHEEL_SPACE_INDEX   = 2;
static constexpr int BACK_RIGHT_WHEEL_SPACE_INDEX  = 3;

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


    /**
     * Ramp the velocity over the given timestep and set the target velocity on the motor.
     *
     * NOTE: This function has no state.
     * Also NOTE: This function handles all electrical rpm to meters/second conversion.
     *
     * @param velocity_target The target velocity in m/s
     * @param velocity_current The current velocity m/s
     * @param time_to_ramp The time allocated for acceleration in seconds
     *
     */
    WheelSpace_t rampWheelVelocity(const WheelSpace_t& current_primitive,
                                   const EuclideanSpace_t& target_euclidean_velocity,
                                   const double& time_to_ramp);

    std::unique_ptr<TbotsProto::DirectControlPrimitive> rampWheelVelocity(
            const std::pair<Vector, AngularVelocity> current_primitive,
            TbotsProto::DirectControlPrimitive& target_velocity_primitive,
            const double& time_to_ramp);

   private:
    /**
     * The radius of the robot in meters.
     */
    const double robot_radius_m_{};
    const RobotConstants_t robot_constants;

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

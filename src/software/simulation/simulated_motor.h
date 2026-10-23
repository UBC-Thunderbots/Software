#pragma once

#include <optional>
#include <random>

#include "proto/primitive.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "shared/robot_constants.h"
#include "software/physics/euclidean_to_wheel.h"
#include "software/time/duration.h"
#include "software/world/robot_state.h"

/**
 * The simulator's stand-in for MotorService.
 *
 * Like MotorService, it ramps the commanded velocity to keep wheel acceleration
 * within limits, and reports the wheel velocities the encoders measure. The measured
 * velocities come from the robot's true state in the simulator with noise added, and
 * are written to the same RobotStatus fields as MotorService.
 */
class SimulatedMotorService
{
   public:
    /**
     * The error added to the true wheel velocities. The defaults are perfect encoders.
     */
    struct NoiseConfig
    {
        // Standard deviation of the white noise added to each wheel velocity, in m/s
        double wheel_velocity_stddev_m_per_s = 0.0;
    };

    /**
     * Creates a new SimulatedMotorService.
     *
     * @param robot_constants The robot constants
     * @param noise_config The error to add to the true wheel velocities
     * @param seed The seed for the noise, so that runs are repeatable
     */
    SimulatedMotorService(const robot_constants::RobotConstants& robot_constants,
                          const NoiseConfig& noise_config, unsigned int seed);

    /**
     * Ramps the primitive's commanded velocity and updates the robot status with the
     * wheel velocities measured this step.
     *
     * Only the drive velocities are ramped. The rest of the primitive (dribbler,
     * kick/chip) passes through unchanged.
     *
     * Only direct velocity control is supported. A primitive without it, such as the
     * empty one PrimitiveExecutor returns when the robot has no primitive, ramps the
     * robot to a stop.
     *
     * @param primitive DirectControlPrimitive to execute
     * @param true_state The robot's true state in the simulator
     * @param robot_status RobotStatus message to modify with the current motor status
     * @param time_elapsed_since_last_poll The time since the last poll
     *
     * @return the primitive with its velocity ramped, to send to the physics simulation
     */
    TbotsProto::DirectControlPrimitive poll(
        const TbotsProto::DirectControlPrimitive& primitive, const RobotState& true_state,
        TbotsProto::RobotStatus& robot_status,
        const Duration& time_elapsed_since_last_poll);

   private:
    /**
     * Reads the wheel velocities the encoders would measure: the robot's true velocity
     * converted to wheel space, with noise added.
     *
     * @param true_state The robot's true state in the simulator
     *
     * @return the measured wheel velocities in m/s
     */
    WheelSpace_t readWheelVelocities(const RobotState& true_state);

    /**
     * Ramps the target wheel velocities toward the primitive's commanded velocity, with
     * the same wheel acceleration limit as MotorService.
     *
     * @param primitive DirectControlPrimitive to execute
     * @param time_elapsed_since_last_poll_s The time since the last poll in seconds
     */
    void updateTargetVelocities(const TbotsProto::DirectControlPrimitive& primitive,
                                double time_elapsed_since_last_poll_s);

    /**
     * Builds the motor status (current and target velocities) and assigns it to
     * robot_status.
     *
     * @param robot_status RobotStatus message to modify with the current motor status
     * @param current_wheel_velocities the measured wheel velocities in m/s
     */
    void updateMotorStatus(TbotsProto::RobotStatus& robot_status,
                           const WheelSpace_t& current_wheel_velocities) const;

    NoiseConfig noise_config_;

    EuclideanToWheel euclidean_to_four_wheel_;

    // The ramped wheel velocities from the last poll, which the next poll ramps from.
    // Empty until the first poll, which starts it from the measured wheel velocities so
    // that the ramp starts from how fast the robot is actually moving
    std::optional<WheelSpace_t> target_wheel_velocities_;

    std::mt19937 random_generator_;

    // Unit normal noise, scaled by the configured standard deviation.
    // std::normal_distribution requires a positive standard deviation, but perfect
    // encoders have a standard deviation of 0
    std::normal_distribution<double> standard_normal_distribution_;
};

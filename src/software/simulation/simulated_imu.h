#pragma once

#include <random>

#include "proto/robot_status_msg.pb.h"
#include "software/world/robot_state.h"

/**
 * The simulator's stand-in for ImuService.
 *
 * Instead of reading the gyroscope, it takes the robot's true angular velocity from
 * the simulator and adds bias and noise. It writes the same RobotStatus fields as
 * ImuService, so the robot localizer uses its readings the same way it does on the
 * real robot.
 *
 * Only angular velocity is simulated, since that is the only IMU reading the robot
 * localizer uses.
 */
class SimulatedImuService
{
   public:
    /**
     * The error added to the true angular velocity. The defaults are a perfect sensor.
     */
    struct NoiseConfig
    {
        // Constant offset added to every reading, in rad/s
        double angular_velocity_bias_rad_per_s = 0.0;

        // Standard deviation of the white noise added to every reading, in rad/s
        double angular_velocity_stddev_rad_per_s = 0.0;
    };

    /**
     * Creates a new SimulatedImuService.
     *
     * @param noise_config The error to add to the true angular velocity
     * @param seed The seed for the noise, so that runs are repeatable
     */
    SimulatedImuService(const NoiseConfig& noise_config, unsigned int seed);

    /**
     * Updates the robot status with an IMU reading of the robot's angular velocity.
     *
     * @param true_state The robot's true state in the simulator
     * @param robot_status The robot status to update
     */
    void poll(const RobotState& true_state, TbotsProto::RobotStatus& robot_status);

   private:
    NoiseConfig noise_config_;

    std::mt19937 random_generator_;

    // Unit normal noise, scaled by the configured standard deviation.
    // std::normal_distribution requires a positive standard deviation, but a perfect
    // sensor has a standard deviation of 0
    std::normal_distribution<double> standard_normal_distribution_;
};

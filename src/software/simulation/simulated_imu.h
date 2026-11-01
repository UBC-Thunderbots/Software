#pragma once

#include <optional>
#include <random>

#include "proto/robot_status_msg.pb.h"
#include "software/geom/angular_velocity.h"
#include "software/geom/vector.h"
#include "software/time/duration.h"
#include "software/world/robot_state.h"

/**
 * The simulator's ImuService.
 *
 * It takes the robot's true motion from the simulator and adds bias and noise. It writes
 * the same RobotStatus fields as ImuService, so the robot localizer uses its readings the
 * same way it does on the real robot.
 *
 * Reports the robot's angular velocity, angular
 * acceleration, and linear acceleration.
 */
class SimulatedImuService
{
   public:
    /**
     * The error added to the true readings.
     */
    struct NoiseConfig
    {
        // Constant offset added to every angular velocity reading, in rad/s
        double angular_velocity_bias_rad_per_s = 0.0;

        // Standard deviation of the white noise added to every angular velocity
        // reading, in rad/s. Angular acceleration is computed from this, so
        // this noise shows up in it too
        double angular_velocity_stddev_rad_per_s = 0.0;

        // Constant offset added to every linear acceleration reading, in the robot's
        // local frame, in m/s^2
        Vector linear_acceleration_bias_m_per_s_2 = Vector();

        // Standard deviation of the white noise added to each axis of every linear
        // acceleration reading, in m/s^2
        double linear_acceleration_stddev_m_per_s_2 = 0.0;
    };

    /**
     * Creates a new SimulatedImuService.
     *
     * @param noise_config The error to add to the true readings
     * @param seed The seed for the noise, so that runs are repeatable
     */
    SimulatedImuService(const NoiseConfig& noise_config, unsigned int seed);

    /**
     * Updates the robot status with IMU readings of the robot's angular velocity,
     * angular acceleration and linear acceleration.
     *
     * @param true_state The robot's true state in the simulator
     * @param robot_status The robot status to update
     * @param time_elapsed_since_last_poll The time since the last poll
     */
    void poll(const RobotState& true_state, TbotsProto::RobotStatus& robot_status,
              const Duration& time_elapsed_since_last_poll);

    /**
     * Reset all readings
     */
    void reset();

   private:
    /**
     * Reads the gyroscope: the robot's true angular velocity, with bias and noise
     * added.
     *
     * @param true_state The robot's true state in the simulator
     *
     * @return the measured angular velocity
     */
    AngularVelocity readAngularVelocity(const RobotState& true_state);

    /**
     * Reads the accelerometer: the change in the robot's true velocity since the
     * previous poll, rotated into the robot's local frame, with bias and noise added.
     *
     * @param true_state The robot's true state in the simulator
     * @param previous_velocity The robot's true global velocity at the previous poll
     * @param time_elapsed_since_last_poll_s The time since the last poll in seconds
     *
     * @return the measured linear acceleration in the robot's local frame, in m/s^2
     */
    Vector readLinearAcceleration(const RobotState& true_state,
                                  const Vector& previous_velocity,
                                  double time_elapsed_since_last_poll_s);

    NoiseConfig noise_config_;

    // The angular velocity reading from the previous poll
    std::optional<AngularVelocity> prev_angular_velocity_;

    // The robot's true global velocity at the previous poll
    std::optional<Vector> prev_velocity_;

    std::mt19937 random_generator_;

    // N(0,1) noise, scaled by the configured standard deviations.
    std::normal_distribution<double> standard_normal_distribution_;
};

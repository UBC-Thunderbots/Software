#pragma once

#include <Eigen/Dense>
#include <deque>
#include <optional>

#include "proto/world.pb.h"
#include "software/embedded/services/imu.h"
#include "software/geom/angle.h"
#include "software/sensor_fusion/filter/kalman_filter.hpp"
#include "software/util/make_enum/make_enum.hpp"

MAKE_ENUM(StateIndex, ORIENTATION, ANGULAR_VELOCITY, ANGULAR_ACCELERATION);

MAKE_ENUM(MeasurementIndex, VISION_ORIENTATION, MOTOR_ANGULAR_VELOCITY,
          IMU_ANGULAR_VELOCITY, TARGET_ANGULAR_ACCELERATION);

/**
 * Estimates robot orientation, angular velocity, and angular acceleration
 * using a Kalman filter.
 *
 * The filter keeps a history of recent predict/update operations. When delayed
 * vision data arrives, the localizer rewinds to the matching historical state,
 * applies the delayed measurement, then replays newer steps to recover the
 * current estimate.
 */
class RobotLocalizer
{
   public:
    /**
     * Creates a new robot localizer.
     *
     * The variances determine how strongly each source influences the estimate.
     *
     * @param process_noise_variance Variance applied to the process noise model.
     * @param vision_noise_variance Variance of camera heading measurements.
     * @param motor_sensor_noise_variance Variance of motor sensor angular velocity.
     * @param target_angular_acceleration_variance Variance of commanded
     * angular acceleration measurements.
     */
    RobotLocalizer(double process_noise_variance, double vision_noise_variance,
                   double motor_sensor_noise_variance,
                   double target_angular_acceleration_variance);

    /**
     * Runs one prediction step using elapsed time since the previous call.
     *
     * @param target_acceleration The target acceleration the robot is trying to attain
     * right now.
     */
    void step(const AngularVelocity& target_acceleration);

    /**
     * Update the orientation from vision.
     *
     * @param orientation Vision reading of the orientation of the robot in world space
     * @param age_seconds Age in seconds of the vision snapshot (time since it was taken)
     */
    void updateVision(const Angle& orientation, double age_seconds);

    /**
     * Update the angular velocity from velocity reported by motor sensors
     * (i.e. encoders or Hall sensors).
     *
     * @param angular_velocity angular velocity of the robot
     */
    void updateMotorSensors(const AngularVelocity& angular_velocity);

    /**
     * Update the angular velocity from IMU.
     *
     * @param angular_velocity angular velocity of the robot
     */
    void updateImu(const AngularVelocity& angular_velocity);

    /**
     * Update the target acceleration.
     *
     * @param angular_acceleration Target angular acceleration of the robot
     */
    void updateTargetAcceleration(const AngularVelocity& angular_acceleration);

    /**
     * Gets estimated orientation of the robot.
     *
     * @return estimated orientation of the robot
     */
    Angle getOrientation() const;

    /**
     * Gets estimated angular velocity of the robot.
     *
     * @return estimated angular velocity of the robot
     */
    AngularVelocity getAngularVelocity() const;

    /**
     * Gets estimated angular acceleration of the robot.
     *
     * @return estimated angular acceleration of the robot
     */
    double getAngularAccelerationRadians() const;

   private:
    /**
     * Update the Kalman filter with the orientation from vision.
     *
     * @param orientation Vision reading of the orientation of the robot in world space
     */
    void updateFilterWithVision(const Angle& orientation);

    static constexpr unsigned int STATE_SIZE = reflective_enum::size<StateIndex>();
    static constexpr unsigned int MEASUREMENT_SIZE =
        reflective_enum::size<MeasurementIndex>();

    /**
     * Snapshot of a Kalman filter predict/update step needed for rollback/replay.
     */
    struct FilterStep
    {
        struct Predict
        {
            Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> process_model;
            Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> process_covariance;
            Eigen::Vector<double, STATE_SIZE> control_model;
            Eigen::Vector<double, 1> control_input;
        };

        struct Update
        {
            Eigen::Matrix<double, MEASUREMENT_SIZE, STATE_SIZE> measurement_model;
            Eigen::Vector<double, MEASUREMENT_SIZE> measurement;
        };

        std::optional<Predict> prediction;
        std::optional<Update> update;

        Eigen::Vector<double, STATE_SIZE> state_estimate;
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> state_covariance;

        std::chrono::time_point<std::chrono::steady_clock> time;
    };

    KalmanFilter<STATE_SIZE, MEASUREMENT_SIZE, 1> filter_;

    // Process noise variance used in prediction
    double process_noise_variance_;

    std::chrono::time_point<std::chrono::steady_clock> last_step_time_;

    // History is ordered newest-first (front is the most recent step)
    std::deque<FilterStep> history;
};

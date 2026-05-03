#pragma once

#include <Eigen/Dense>
#include <deque>
#include <optional>

#include "proto/world.pb.h"
#include "software/embedded/services/imu.h"
#include "software/geom/angle.h"
#include "software/geom/point.h"
#include "software/geom/vector.h"
#include "software/sensor_fusion/filter/kalman_filter.hpp"
#include "software/util/make_enum/make_enum.hpp"

MAKE_ENUM(StateIndex, X_POSITION, Y_POSITION, ORIENTATION, X_VELOCITY, Y_VELOCITY,
          ANGULAR_VELOCITY);

MAKE_ENUM(MeasurementIndex, VISION_X_POSITION, VISION_Y_POSITION, VISION_ORIENTATION,
          VISION_X_VELOCITY, VISION_Y_VELOCITY, VISION_ANGULAR_VELOCITY, MOTOR_X_VELOCITY,
          MOTOR_Y_VELOCITY, MOTOR_ANGULAR_VELOCITY, IMU_ANGULAR_VELOCITY);

MAKE_ENUM(ControlIndex, X_ACCELERATION, Y_ACCELERATION, ANGULAR_ACCELERATION);

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
     */
    explicit RobotLocalizer(double process_noise_variance, double vision_noise_variance,
                            double motor_sensor_noise_variance);

    /**
     * Runs one prediction step using elapsed time since the previous call.
     *
     * @param target_linear_acceleration The current target linear acceleration of the
     * robot
     * @param target_angular_acceleration The current target angular acceleration of the
     * robot
     */
    void step(const Vector& target_linear_acceleration,
              const AngularVelocity& target_angular_acceleration);

    /**
     * Update the robot's position and velocity from data reported by vision.
     *
     * @param position Vision reading of the robot's position in world space
     * @param velocity Vision reading of the robot's velocity in world space
     * @param orientation Vision reading of the robot's orientation in world space
     * @param angular_velocity Vision reading of the robot's angular velocity
     * @param age_seconds Age in seconds of the vision snapshot (time since it was taken)
     */
    void updateVision(const Point& position, const Vector& velocity,
                      const Angle& orientation, const AngularVelocity& angular_velocity,
                      double age_seconds);

    /**
     * Update the robot's velocity from data reported by motor sensors
     * (i.e. encoders or Hall sensors).
     *
     * @param velocity Motor sensor reading of the robot's velocity in local space
     * @param angular_velocity Motor sensor reading of the robot's angular velocity
     */
    void updateMotorSensors(const Vector& velocity,
                            const AngularVelocity& angular_velocity);

    /**
     * Update the angular velocity from IMU.
     *
     * @param angular_velocity angular velocity of the robot
     */
    void updateImu(const AngularVelocity& angular_velocity);

    /**
     * Gets the estimated position of the robot in world space.
     *
     * @return the estimated position of the robot in world space
     */
    Point getPosition() const;

    /**
     * Gets the estimated velocity of the robot in world space.
     *
     * @return the estimated velocity of the robot in world space
     */
    Vector getVelocity() const;

    /**
     * Gets the estimated orientation of the robot in world space.
     *
     * @return estimated orientation of the robot in world space
     */
    Angle getOrientation() const;

    /**
     * Gets the estimated angular velocity of the robot.
     *
     * @return estimated angular velocity of the robot
     */
    AngularVelocity getAngularVelocity() const;

   private:
    /**
     * Update the Kalman filter with the robot's position and velocity from vision.
     *
     * @param position Vision reading of the robot's position in world space
     * @param velocity Vision reading of the robot's velocity in world space
     * @param orientation Vision reading of the robot's orientation in world space
     * @param angular_velocity Vision reading of the robot's angular velocity
     */
    void updateFilterWithVision(const Point& position, const Vector& velocity,
                                const Angle& orientation,
                                const AngularVelocity& angular_velocity);

    static constexpr size_t STATE_SIZE = reflective_enum::size<StateIndex>();
    static constexpr size_t MEASUREMENT_SIZE = reflective_enum::size<MeasurementIndex>();
    static constexpr size_t CONTROL_SIZE = reflective_enum::size<ControlIndex>();

    /**
     * Snapshot of a Kalman filter predict/update step needed for rollback/replay.
     */
    struct FilterStep
    {
        struct Predict
        {
            Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> process_model;
            Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> process_covariance;
            Eigen::Matrix<double, STATE_SIZE, CONTROL_SIZE> control_model;
            Eigen::Vector<double, CONTROL_SIZE> control_input;
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

    KalmanFilter<STATE_SIZE, MEASUREMENT_SIZE, CONTROL_SIZE> filter_;

    // Process noise variance used in prediction
    double process_linear_acceleration_noise_variance_;
    double process_angular_acceleration_noise_variance_;

    std::chrono::time_point<std::chrono::steady_clock> last_step_time_;

    // History is ordered newest-first (front is the most recent step)
    std::deque<FilterStep> history;
};

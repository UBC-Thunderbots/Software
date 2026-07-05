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
          MOTOR_X_VELOCITY, MOTOR_Y_VELOCITY, MOTOR_ANGULAR_VELOCITY,
          IMU_ANGULAR_VELOCITY);

MAKE_ENUM(ControlIndex, X_ACCELERATION, Y_ACCELERATION);

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
    struct VisionData
    {
        Point position;
        Angle orientation;
        double age_seconds;
    };

    struct MotorData
    {
        Vector velocity;
        AngularVelocity angular_velocity;
    };

    struct ImuData
    {
        AngularVelocity angular_velocity;
    };

    struct RobotLocalizerConfig
    {
        double process_noise_variance;
        double vision_noise_variance;
        double motor_sensor_noise_variance;
    };

    /**
     * Creates a new robot localizer.
     *
     * The variances determine how strongly each source influences the estimate.
     *
     * @param config Configuration for the localizer variances.
     */
    explicit RobotLocalizer(const RobotLocalizerConfig& config);

    /**
     * Runs one prediction step using elapsed time since the previous call.
     *
     * @param linear_acceleration The current linear acceleration of the robot
     */
    void step(const Vector& linear_acceleration);

    /**
     * Update the robot's position and orientation from data reported by vision.
     *
     * @param data Vision reading of the robot's position, orientation and age
     */
    void update(const VisionData& data);

    /**
     * Update the robot's velocity from data reported by motor sensors
     * (i.e. encoders or Hall sensors).
     *
     * @param data Motor sensor reading of the robot's velocity and angular velocity
     */
    void update(const MotorData& data);

    /**
     * Update the angular velocity from IMU.
     *
     * @param data IMU reading of the robot's angular velocity
     */
    void update(const ImuData& data);

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
     * Update the Kalman filter with the robot's position and orientation from vision.
     *
     * @param position Vision reading of the robot's position in world space
     * @param orientation Vision reading of the robot's orientation in world space
     */
    void updateFilterWithVision(const Point& position, const Angle& orientation);

    static constexpr size_t STATE_SIZE       = reflective_enum::size<StateIndex>();
    static constexpr size_t MEASUREMENT_SIZE = reflective_enum::size<MeasurementIndex>();
    static constexpr size_t CONTROL_SIZE     = reflective_enum::size<ControlIndex>();

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

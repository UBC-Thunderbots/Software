#pragma once

#include <Eigen/Dense>
#include <deque>
#include <optional>

#include "proto/primitive.pb.h"
#include "proto/robot_status_msg.pb.h"
#include "software/embedded/services/imu.h"
#include "software/geom/angle.h"
#include "software/geom/point.h"
#include "software/geom/vector.h"
#include "software/sensor_fusion/filter/kalman_filter.hpp"
#include "software/time/duration.h"
#include "software/util/make_enum/make_enum.hpp"
#include "software/world/robot_state.h"

MAKE_ENUM(StateIndex, X_POSITION, Y_POSITION, ORIENTATION, X_VELOCITY, Y_VELOCITY,
          ANGULAR_VELOCITY);

MAKE_ENUM(MeasurementIndex, VISION_X_POSITION, VISION_Y_POSITION, VISION_ORIENTATION,
          MOTOR_X_VELOCITY, MOTOR_Y_VELOCITY, MOTOR_ANGULAR_VELOCITY,
          IMU_ANGULAR_VELOCITY);

MAKE_ENUM(ControlIndex, X_VELOCITY_TARGET, Y_VELOCITY_TARGET);

MAKE_ENUM(FilterStepType, PREDICT, MOTOR_DATA, IMU_DATA, VISION_DATA);

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
     * Runs one prediction step over the given elapsed time.
     *
     * @param target_velocity The global-frame linear velocity the robot is currently
     * being commanded to achieve
     * @param delta_time The elapsed time since the previous step
     */
    void predict(const Vector& target_velocity, const Duration& delta_time);

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

    /**
     * Gets the current robot state estimate.
     *
     * @return The estimated robot state
     */
    RobotState getRobotState() const;

   private:
    /**
     * Update the Kalman filter with the robot's position and orientation from vision.
     *
     * @param position Vision reading of the robot's position in world space
     * @param orientation Vision reading of the robot's orientation in world space
     */
    void updateFilterWithVision(const Point& position, const Angle& orientation);

    /**
     * Computes the process model, process covariance, and control model for the
     * given elapsed time, and writes them into the filter. Does not run the
     * predict step itself.
     *
     * @param delta_time_seconds The elapsed time to generate the prediction
     * matrices for
     */
    void generatedPredictionMatrices(double delta_time_seconds);

    /**
     * Writes the measurement model for the given data source into the filter.
     *
     * @param source Which sensor's measurement model to generate. Must not be
     * FilterStepType::PREDICT.
     */
    void generateMeasurementModel(FilterStepType source);

    static constexpr size_t STATE_SIZE       = reflective_enum::size<StateIndex>();
    static constexpr size_t MEASUREMENT_SIZE = reflective_enum::size<MeasurementIndex>();
    static constexpr size_t CONTROL_SIZE     = reflective_enum::size<ControlIndex>();

    /**
     * Snapshot of a Kalman filter predict/update step needed for rollback/replay.
     */
    struct FilterStep
    {
        FilterStepType type;

        // Set iff type == PREDICT. process_model/process_covariance/control_model are
        // recomputed from the elapsed time during replay instead of being stored (see
        // generatedPredictionMatrices).
        std::optional<Eigen::Vector<double, CONTROL_SIZE>> control_input;

        // Set iff type != PREDICT. The measurement model is regenerated from type
        // during replay (see generateMeasurementModel).
        std::optional<Eigen::Vector<double, MEASUREMENT_SIZE>> measurement;

		// Post operation state
        Eigen::Vector<double, STATE_SIZE> state_estimate;
        Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> state_covariance;

        double time_seconds;
    };

    KalmanFilter<STATE_SIZE, MEASUREMENT_SIZE, CONTROL_SIZE> filter_;

    // Process noise variance used in prediction. The linear term models how much
    // actual velocity deviates from the commanded target velocity (a rate, per unit
    // time); the angular term models unmeasured angular acceleration disturbance.
    double process_linear_velocity_noise_variance_;
    double process_angular_acceleration_noise_variance_;

    // History is ordered newest-first (front is the most recent step)
    std::deque<FilterStep> history;

    double current_time_seconds_ = 0.0;
};

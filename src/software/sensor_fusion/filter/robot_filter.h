#pragma once

#include <optional>
#include <vector>

#include "software/geom/angle.h"
#include "software/geom/point.h"
#include "software/sensor_fusion/filter/kalman_filter.hpp"
#include "software/sensor_fusion/filter/vision_detection.h"
#include "software/time/timestamp.h"
#include "software/world/robot.h"

class RobotFilter
{
   public:
    /**
     * Creates a new robot filter
     *
     * @param current_robot_state the data of current state of the robot
     * @param expiry_buffer_duration the time when the robot is determined to be removed
     * from the field if data about the robot is not received before that time
     */
    explicit RobotFilter(Robot current_robot_state);
    explicit RobotFilter(RobotDetection current_robot_state);

    /**
     * Update the filter with the new SSLRobot detections, and returns the new
     * estimated state of the robot given the new data. 
     * 
     *
     * @param new_robot_data A list of SSLRobot detections containing new robot data.
     * The data does not all have to be for a particular Robot, the filter will only use
     * the new Robot data that matches the robot id the filter was constructed with.
     * @param current_time The time to estimate the robot's state at
     * @param breakbeam_tripped_id The id of the robot with the tripped breakbeam
     * according to sensor fusion filtering logic (or none if no robot has a tripped
     * beam).
     * 
     * @return The new Robot based on the estimated state of the Robot given the new data.
     * If there is no robot data for this robot, it will return the prediction of the filter
     * for a few updates, but if there are (EXPIRED_FRAME_THRESHOLD) consecutive missing frames,
     * returns std::nullopt
     */
    std::optional<Robot> estimateRobotState(
        const std::vector<RobotDetection>& new_robot_data, const Timestamp& current_time, const std::optional<RobotId> breakbeam_tripped_id = std::nullopt);

    /**
     * Returns the id of the Robot that this filter is filtering for
     *
     * @return the id of the Robot that this filter is filtering for
     */
    unsigned int getRobotId() const;

   private:
    Robot current_robot_state;

	// KF Dimensions
	// Position State: position x, position y, velocity x, velocity y
    // Angle State: angle theta, angular velocity w
    static constexpr int POS_STATE_SIZE       = 4;
    static constexpr int ANG_STATE_SIZE       = 2;
	// Position Measurement: x and y from vision
    // Angle Measurement: theta from vision
    static constexpr int POS_MEASUREMENT_SIZE = 2;
    static constexpr int ANG_MEASUREMENT_SIZE = 1;
	// No control 
    static constexpr int CONTROL_SIZE         = 1;

    // Counter to keep track of revolutions, to unwrap to feed to Kalman filter
    int revolutions = 0;

    using PosKalmanFilter = KalmanFilter<POS_STATE_SIZE, POS_MEASUREMENT_SIZE, CONTROL_SIZE>;
    using AngKalmanFilter = KalmanFilter<ANG_STATE_SIZE, ANG_MEASUREMENT_SIZE, CONTROL_SIZE>;

    // Will be keeping Position and Angle in double, in units of metres and radians respectively
    using PosMeasurement  = Eigen::Vector<double, POS_MEASUREMENT_SIZE>;
    using AngMeasurement  = Eigen::Vector<double, ANG_MEASUREMENT_SIZE>;

    /**
     * Returns the detection we should treat as the robot this frame, which is the
     * highest confidence detection. 
     *
     * @param new_robot_detections The detections to choose from
     *
     * @return The detection to use, or std::nullopt if there is no usable detection
     */
    std::optional<RobotDetection> getBestRobotDetection(
        const std::vector<RobotDetection>& new_robot_detections);

    /**
     * Advances the Kalman filter's estimate forward to the given time using a constant
     * velocity motion model.
     *
     * Both the motion model and the process noise depend on how much time is being
     * advanced over, so both are rebuilt here rather than being fixed at construction.
     *
     * @param delta_t The amount of time to advance the estimate by, in seconds
     */
    void predict(double delta_t);

    /**
     * Returns whether the robot could physically have reached the given position since
     * the last accepted detection, assuming it cannot exceed the maximum robot speed.
     *
     * @param detection_position The position of the detection to check
     * @param current_time The time the detection was taken at
     *
     * @return whether the detection is within reach of the current estimate
     */
    bool isWithinMaxRobotSpeed(const Point& detection_position,
                              const Timestamp& current_time) const;

    /**
     * Discards the filter's current estimate and reinitializes it on the given
     * measurement, at rest and with the covariance widened back out.
     *
     * @param measurement The measurement to reinitialize the estimate on
     * @param current_time The time the measurement was taken at
     */
    void reset(const PosMeasurement& pos_measurement, const AngMeasurement& ang_measurement, const Timestamp& current_time);

    PosKalmanFilter pos_kalman_filter;
    AngKalmanFilter ang_kalman_filter;
    std::optional<Timestamp> prev_detection_timestamp;
    std::optional<PosMeasurement> prev_pos_measurement;
    std::optional<AngMeasurement> prev_ang_measurement;
    std::optional<Timestamp> last_predict_timestamp;
    int consecutive_outliers;
    int expired_frame_count;
};

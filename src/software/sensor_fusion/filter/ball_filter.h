#pragma once

#include <Eigen/Dense>
#include <optional>

#include "software/geom/point.h"
#include "software/geom/rectangle.h"
#include "software/world/field.h"
#include "software/sensor_fusion/filter/kalman_filter.hpp"
#include "software/sensor_fusion/filter/vision_detection.h"
#include "software/time/timestamp.h"
#include "software/world/ball.h"
#include "software/world/robot.h"

class BallFilter
{
   public:

    /**
     * Creates a new Ball Filter
     */
    explicit BallFilter();

    /**
     * Update the filter with the new ball detection data, and returns the new
     * estimated state of the ball given the new data
     *
     * @param new_ball_detections A list of new Ball detections
     * @param field The field being played on. Detections outside its boundary are
     * ignored, and its goals are obstacles the ball may bounce off.
     * @param robots The robots currently on the field, which the ball may bounce off
     * @param current_time The time to estimate the ball's state at
     *
     * @return The new ball based on the estimated state of the ball given the new data.
     * If a filtered result cannot be calculated, returns std::nullopt
     */
    std::optional<Ball> estimateBallState(
        const std::vector<BallDetection>& new_ball_detections, const Field& field,
        const std::vector<Robot>& robots, const Timestamp& current_time);

   private:

	// KF Dimensions
	// State: position x, position y, veloity x, velocity y
    static constexpr int STATE_SIZE       = 4;
	// Measurement: x and y from vision
    static constexpr int MEASUREMENT_SIZE = 2;
	// No control 
    static constexpr int CONTROL_SIZE     = 1;

    using BallKalmanFilter =
        KalmanFilter<STATE_SIZE, MEASUREMENT_SIZE, CONTROL_SIZE>;
    using Measurement = Eigen::Vector<double, MEASUREMENT_SIZE>;

    /**
     * Returns the detection we should treat as the ball this frame, which is the
     * highest confidence detection lying inside the filter area.
     *
     * @param new_ball_detections The detections to choose from
     * @param filter_area The area within which the ball filter will work. Any detections
     * outside of this area will be ignored.
     *
     * @return The detection to use, or std::nullopt if there is no usable detection
     */
    static std::optional<BallDetection> getBestBallDetection(
        const std::vector<BallDetection>& new_ball_detections,
        const Rectangle& filter_area);

    /**
     * Advances the Kalman filter's estimate forward to the given time using a constant
     * velocity motion model with damping.
     *
     * Both the motion model and the process noise depend on how much time is being
     * advanced over, so both are rebuilt here rather than being fixed at construction.
     *
     * @param delta_t The amount of time to advance the estimate by, in seconds
     */
    void predict(double delta_t);

    /**
     * Widens the covariance if the ball is in contact with anything on the field -- a
     * robot, a goalpost, the back of a net, or the walls around the field.
     *
     * The check is against the whole path the ball travelled this frame rather than only
     * where it ended up. A ball moving at 5 m/s covers over 8 cm between frames at 60 Hz,
     * so a test that only asked whether the ball was currently within its own radius of a
     * surface would step straight over anything thin, and a goalpost is thin.
     *
     * Widening is the whole response. A ball touching another object is no longer
     * described by the motion model, and the filter has no business staying as confident
     * as it was -- but what the contact did to the velocity is not knowable from geometry,
     * so we let the following detections settle it rather than asserting a bounce.
     *
     * @param previous_position Where the estimate was before it was advanced this frame
     * @param robots The robots currently on the field
     * @param field The field being played on
     */
    void widenCovarianceOnContact(const Point& previous_position,
                                  const std::vector<Robot>& robots,
                                  const Field& field);

    /**
     * Returns whether the ball could physically have reached the given position since
     * the last accepted detection, assuming it cannot exceed the maximum ball speed.
     *
     * @param detection_position The position of the detection to check
     * @param current_time The time the detection was taken at
     *
     * @return whether the detection is within reach of the current estimate
     */
    bool isWithinMaxBallSpeed(const Point& detection_position,
                              const Timestamp& current_time) const;

    /**
     * Discards the filter's current estimate and reinitializes it on the given
     * measurement, at rest and with the covariance widened back out.
     *
     * @param measurement The measurement to reinitialize the estimate on
     * @param current_time The time the measurement was taken at
     */
    void reset(const Measurement& measurement, const Timestamp& current_time);

    BallKalmanFilter kalman_filter;
    int consecutive_outliers;
    std::optional<Timestamp> prev_detection_timestamp;
    std::optional<Measurement> prev_measurement;
    std::optional<Timestamp> last_predict_timestamp;
	
};

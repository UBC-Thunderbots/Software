#pragma once

#include <Eigen/Dense>
#include <optional>

#include "software/geom/point.h"
#include "software/geom/rectangle.h"
#include "software/sensor_fusion/filter/kalman_filter.hpp"
#include "software/sensor_fusion/filter/vision_detection.h"
#include "software/time/timestamp.h"
#include "software/world/ball.h"

/**
 * Given ball data from SSL Vision, filters and returns the position/velocity of the
 * "real" ball.
 *
 * This filter tracks the ball with a Kalman filter over the state
 * [x, y, x_velocity, y_velocity], using a constant velocity motion model with a damping
 * term to account for the ball decelerating due to friction. Each frame it takes the
 * highest confidence detection inside the filter area as the measurement.
 *
 * The data we receive isn't perfect, which is why we have a filter. Detections that
 * disagree too strongly with the current estimate (measured by Mahalanobis distance,
 * which accounts for how confident the filter currently is) are rejected as outliers
 * rather than dragging the estimate off the ball's real trajectory. This keeps the
 * output steady, which matters because small deviations in velocity orientation have
 * large effects when the AI predicts the ball's future position. For example,
 * consistently receiving a pass relies on the ball's velocity being stable, otherwise
 * the robot would "jiggle" back and forth as the estimated receiver position kept
 * changing.
 *
 * Rejecting outliers forever would leave the filter stuck if the ball genuinely
 * teleports (a ball placement, or a detection we had wrongly locked onto), so after
 * enough consecutive rejections the filter resets onto the newest detection.
 */
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
     * @param filter_area The area within which the ball filter will work. Any detections
     * outside of this area will be ignored.
     * @param current_time The time to estimate the ball's state at
     *
     * @return The new ball based on the estimated state of the ball given the new data.
     * If a filtered result cannot be calculated, returns std::nullopt
     */
    std::optional<Ball> estimateBallState(
        const std::vector<BallDetection>& new_ball_detections,
        const Rectangle& filter_area, const Timestamp& current_time);

   private:
    // The dimensions of the Kalman filter this ball filter is built on.
    // The state is [x, y, x_velocity, y_velocity], the measurement is [x, y], and
    // there is no control input (we cannot command the ball).
    static constexpr int STATE_SIZE       = 4;
    static constexpr int MEASUREMENT_SIZE = 2;
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
     * @param delta_t The amount of time to advance the estimate by, in seconds
     */
    void predict(double delta_t);

    /**
     * Discards the filter's current estimate and reinitializes it on the given
     * measurement, seeding the velocity from the previous measurement if we have one.
     *
     * @param measurement The measurement to reinitialize the estimate on
     * @param current_time The time the measurement was taken at
     */
    void reset(const Measurement& measurement, const Timestamp& current_time);

    BallKalmanFilter kalman_filter;
    // How many detections in a row have been rejected as outliers
    int consecutive_outliers;
    // The time and value of the most recent accepted measurement. Both are unset until
    // the filter has seen its first usable detection.
    std::optional<Timestamp> prev_detection_timestamp;
    std::optional<Measurement> prev_measurement;
};

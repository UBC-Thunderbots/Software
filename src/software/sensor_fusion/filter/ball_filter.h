#pragma once

#include <boost/circular_buffer.hpp>
#include <optional>

#include "software/geom/line.h"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"
#include "software/sensor_fusion/filter/vision_detection.h"
#include "software/time/timestamp.h"
#include "software/world/ball.h"
#include "software/world/timestamped_ball_state.h"

/**
 * Given ball data from SSL Vision, filters for and returns the position/velocity of the
 * "real" ball
 */
class BallFilter
{
   public:
    // The min and max sizes of the ball detection buffer
    static constexpr unsigned int MIN_BUFFER_SIZE = 4;
    static constexpr unsigned int MAX_BUFFER_SIZE = 10;
    // If the estimated ball speed is less than this value, the largest possible buffer
    // will be used by the filter
    static constexpr double MIN_BUFFER_SIZE_VELOCITY_MAGNITUDE = 0.5;
    // If the estimated ball speed is greater than this value, the smallest possible
    // buffer will be used by the filter
    static constexpr double MAX_BUFFER_SIZE_VELOCITY_MAGNITUDE = 4.0;
    // The extra amount beyond the ball's max speed that we treat ball detections as valid
    static constexpr double MAX_ACCEPTABLE_BALL_SPEED_BUFFER = 2.0;

    /**
     * Creates a new Ball Filter
     *
     * @param filter_area The area within which the ball filter will work. Any detections
     * outside of this area will be ignored.
     */
    explicit BallFilter(const Rectangle& filter_area);

    /**
     * Filters the new ball detection data, and returns the updated state of the ball
     * given the new data *
     * @param current_ball_state The current state of the Ball
     * @param new_ball_detections A list of new Ball detections
     *
     * @return The updated state of the ball given the new data
     */
    std::optional<TimestampedBallState> estimateBallState(
        const std::vector<BallDetection> &new_ball_detections);


private:
/**
 * A simple struct we use to pass around velocity estimate data
 */
    struct BallVelocityEstimate
    {
        Vector average_velocity;
        double average_velocity_magnitude;
        // The average of the max velocity magnitude and min velocity magnitude
        double min_max_magnitude_average;
    };

/**
 * A simple struct to pass around linear regression data
 */
    struct LinearRegressionResults
    {
        Line regression_line;
        double regression_error;
    };

    /**
     * Adds ball detections to the buffer stored by this filter. This function will ignore
     * data that is outside of the field, and data that is too far away from the current
     * known ball position and therefore is likely to be random noise.
     *
     * @param new_ball_detections The ball detections to try add to the buffer
     * @param field The field being played on.
     */
    void addNewDetectionsToBuffer(std::vector<BallDetection> new_ball_detections);

    /**
     * Returns how large the buffer of ball detections should be based on the ball's
     * estimated velocity. A slower moving ball will result in a larger buffer size, and a
     * faster ball will result in a smaller buffer size. This is because with a slow
     * moving ball, we need more data in order to fit a line with reasonable accuracy,
     * since the datapoints will be very close to one another.
     *
     * @param ball_detections The full list of ball detections
     * @return The size the buffer should be to perform filtering operations. If an error
     * occurs that prevents the size from being calculated correctly, returns std::nullopt
     */
    static std::optional<size_t> getAdjustedBufferSize(
            boost::circular_buffer<BallDetection> ball_detections);


    /**
     * Given a list of ball detections, use linear regression to find a line of best fit
     * through the ball positions, and calculate the error of this regression.
     *
     * @param ball_detections The ball detections to use in the regression
     * @return A struct containing the regression line and error of the linear regression
     */
    static LinearRegressionResults getLinearRegressionLine(
            boost::circular_buffer<BallDetection> ball_detections);

    /**
     * Estimates the ball's velocitybased on the current detections in the given buffer.
     * If the ball_regression_line is provided, the detection positions are projected onto
     * the line before the velocities are calculated. If no velocity can be estimated,
     * std::nullopt is returned.
     *
     * @param ball_detections The ball detections to use to calculate
     * @param ball_regression_line The ball_regression_line to snap detections to before
     * calculating velocities.
     * @return A struct containing various estimates of the ball's velocity based on the
     * given detections. If no velocity can be estimated, std::nullopt is returned
     */
    static std::optional<BallVelocityEstimate> estimateBallVelocity(
        boost::circular_buffer<BallDetection> ball_detections,
        const std::optional<Line> &ball_regression_line = std::nullopt);

    /**
     * Uses linear regression to filter the given list of ball detections to fine the
     * current "real" state of the ball.
     *
     * @param ball_detections The detections to filter
     * @return The filtered current state of the ball. If a filtered result cannot be
     * calculated, returns std::nullopt
     */
    static std::optional<TimestampedBallState> estimateBallState(
        boost::circular_buffer<BallDetection> ball_detections);

    Rectangle filter_area;

    // A circular buffer used to store previous ball detections, so we can use them
    // in the filter
    boost::circular_buffer<BallDetection> ball_detection_buffer;
};

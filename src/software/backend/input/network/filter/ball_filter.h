#pragma once

#include <boost/circular_buffer.hpp>
#include <optional>

#include "software/new_geom/line.h"
#include "software/new_geom/point.h"
#include "software/sensor_fusion/ball_detection.h"
#include "software/time/timestamp.h"
#include "software/world/ball.h"
#include "software/world/field.h"
#include "software/world/timestamped_ball_state.h"

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
 * Given ball data from SSL Vision, filters for and returns the position/velocity of the
 * "real" ball
 */
class BallFilter
{
   public:
    // The default min and max sizes of the ball detection buffer
    static constexpr unsigned int DEFAULT_MIN_BUFFER_SIZE = 4;
    static constexpr unsigned int DEFAULT_MAX_BUFFER_SIZE = 10;
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
     * @param min_buffer_size The minimum size of the buffer the filter will use to filter
     * the ball. The buffer will shrink to this size as the ball speeds up
     * @param min_buffer_size The maximum size of the buffer the filter will use to filter
     * the ball. The buffer will grow to this size as the ball slows down
     */
    explicit BallFilter(unsigned int min_buffer_size = DEFAULT_MIN_BUFFER_SIZE,
                        unsigned int max_buffer_size = DEFAULT_MAX_BUFFER_SIZE);

    /**
     * Filters the new ball detection data, and returns the updated state of the ball
     * given the new data *
     * @param current_ball_state The current state of the Ball
     * @param new_ball_detections A list of new Ball detections
     *
     * @return The updated state of the ball given the new data
     */
    std::optional<Ball> getFilteredData(
        const std::vector<BallDetection> &new_ball_detections, const Field &field);

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
    std::optional<size_t> getAdjustedBufferSize(
        boost::circular_buffer<BallDetection> ball_detections);

    /**
     * Adds ball detections to the buffer stored by this filter. This function will ignore
     * data that is outside of the field, and data that is too far away from the current
     * known ball position and therefore is likely to be random noise.
     *
     * @param new_ball_detections The ball detections to try add to the buffer
     * @param field The field being played on.
     */
    void addNewDetectionsToBuffer(std::vector<BallDetection> new_ball_detections,
                                  const Field &field);

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
    std::optional<BallVelocityEstimate> estimateBallVelocity(
        boost::circular_buffer<BallDetection> ball_detections,
        const std::optional<Line> &ball_regression_line = std::nullopt);

    /**
     * Given a list of ball detections, use linear regression to find a line of best fit
     * through the ball positions, and calculate the error of this regression.
     *
     * @param ball_detections The ball detections to use in the regression
     * @return A struct containing the regression line and error of the linear regression
     */
    LinearRegressionResults getLinearRegressionLine(
        boost::circular_buffer<BallDetection> ball_detections);

    /**
     * Uses linear regression to filter the given list of ball detections to fine the
     * current "real" state of the ball.
     *
     * @param ball_detections The detections to filter
     * @return The filtered current state of the ball. If a filtered result cannot be
     * calculated, returns std::nullopt
     */
    std::optional<TimestampedBallState> estimateBallState(
        boost::circular_buffer<BallDetection> ball_detections);

   private:
    unsigned int _min_buffer_size;
    unsigned int _max_buffer_size;

    // A circular buffer used to store previous ball detections, so we can use them
    // in the filter
    boost::circular_buffer<BallDetection> ball_detection_buffer;
};

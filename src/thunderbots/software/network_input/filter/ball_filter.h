#pragma once

#include <boost/circular_buffer.hpp>
#include "ai/world/ball.h"
#include "ai/world/field.h"
#include "geom/point.h"
#include "util/time/timestamp.h"
#include <optional>
#include "geom/line.h"

/**
 * A lightweight datatype used to input new data into the filter.
 * We do this rather than taking the SSL_DetectionBall directly
 * so we can make this module more generic and abstract away
 * the protobuf for testing
 */
struct SSLBallDetection {
    // The position of the ball detection on the field, in meters
    Point position;
    // The timestamp of the detection. This is the timestamp for when the camera frame
    // containing the detection was captured
    Timestamp timestamp;

    bool operator<(const SSLBallDetection &b) const {
        return timestamp < b.timestamp;
    }
};

/**
 * A simple struct we use to pass around velocity estimate data
 */
struct BallVelocityEstimate {
    Vector average_velocity;
    double average_velocity_magnitude;
    // The average of the max velocity magnitude and min velocity magnitude
    double min_max_magnitude_average;
};

/**
 * A simple struct to pass around linear regression data
 */
struct LinearRegressionResults {
    Line regression_line;
    double regression_error;
};

/**
 * Given ball data from SSL Vision, filters for and returns the position/velocity of the
 * "real" ball
 */
class BallFilter {
public:
    /**
     * Creates a new Ball Filter
     *
     * @param min_buffer_size The minimum size of the buffer the filter will use to filter the ball. The buffer will
     * shrink to this size as the ball speeds up
     * @param min_buffer_size The maximum size of the buffer the filter will use to filter the ball. The buffer will
     * grow to this size as the ball slows down
     */
    explicit BallFilter(unsigned int min_buffer_size, unsigned int max_buffer_size);

    /**
     * Filters the new ball detection data, and returns the updated state of the ball
     * given the new data *
     * @param current_ball_state The current state of the Ball
     * @param new_ball_detections A list of new SSL Ball detections
     *
     * @return The updated state of the ball given the new data
     */
    std::optional<Ball> getFilteredData(const std::vector<SSLBallDetection> &new_ball_detections, const Field &field);

    /**
     * Returns how large the buffer of ball detections should be based on the ball's estimated velocity. A slower
     * moving ball will result in a larger buffer size, and a faster ball will result in a smaller buffer size. This
     * is because with a slow moving ball, we need more data in order to fit a line with reasonable accuracy, since the
     * datapoints will be very close to one another.
     *
     * @param ball_detections The full list of ball detections
     * @return The size the buffer should be to perform filtering operations
     */
    size_t getAdjustedBufferSize(boost::circular_buffer<SSLBallDetection> ball_detections);

    /**
     * Adds ball detections to the buffer stored by this filter. This function will ignore data that is outside of the
     * field, and data that is too far away from the current known ball position and therefore is likely to be random
     * noise.
     *
     * @param new_ball_detections The ball detections to try add to the buffer
     * @param field The field being played on.
     */
    void addNewDetectionsToBuffer(std::vector<SSLBallDetection> new_ball_detections, const Field &field);

    /**
     * Estimates the ball's velocitybased on the current detections in the given buffer. If the ball_regression_line
     * is provided, the detection positions are projected onto the line before the velocities are calculated.
     *
     * @param ball_detections The ball detections to use to calculate
     * @param ball_regression_line The ball_regression_line to snap detections to before calculating velocities.
     * @return A struct containing various estimates of the ball's velocity based on the given detections
     */
    BallVelocityEstimate estimateBallVelocity(boost::circular_buffer<SSLBallDetection> ball_detections,
                                              const std::optional<Line> &ball_regression_line = std::nullopt);

    /**
     * Given a list of ball detections, use linear regression to find a line of best fit through the ball positions,
     * and calculate the error of this regression.
     *
     * @param ball_detections The ball detections to use in the regression
     * @return A struct containing the regression line and error of the linear regression
     */
    LinearRegressionResults getLinearRegressionLine(boost::circular_buffer<SSLBallDetection> ball_detections);

    /**
     * Uses linear regression to filter the given list of ball detections to fine the current "real" state of the ball.
     *
     * @param ball_detections The detections to filter
     * @return The filtered current state of the ball
     */
    Ball getLinearRegressionPositionAndVelocity(boost::circular_buffer<SSLBallDetection> ball_detections);

private:
    // A circular buffer used to store previous ball detections, so we can use them
    // in the filter
    boost::circular_buffer<SSLBallDetection> ball_detection_buffer;
    unsigned int _min_buffer_size;
    unsigned int _max_buffer_size;
    // If the estimated ball speed is greater than this value, the smallest possible buffer will be used by the filter
    double MAX_BUFFER_SIZE_VELOCITY_MAGNITUDE = 4.0;
    // If the estimated ball speed is less than this value, the largest possible buffer will be used by the filter
    double MIN_BUFFER_SIZE_VELOCITY_MAGNITUDE = 0.5;
};

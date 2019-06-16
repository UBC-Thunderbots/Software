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
struct SSLBallDetection
{
    // The position of the ball detection on the field, in meters
    Point position;
    // The timestamp of the detection. This is the timestamp for when the camera frame
    // containing the detection was captured
    Timestamp timestamp;

    bool operator<(const SSLBallDetection& b) const {
        return timestamp < b.timestamp;
    }
};

// TODO: comment
struct BallVelocityEstimate
{
    double average_velocity_magnitude;
    double min_max_magnitude_average;
};

struct LinearRegressionResults {
    Line regression_line;
    double regression_error;
};

// TODO: COMMENTS AND ASCII ART
/**
 * Given ball data from SSL Vision, filters for and returns the position/velocity of the
 * "real" ball
 */
class BallFilter
{
   public:
    /**
     * Creates a new Ball Filter
     */
    explicit BallFilter(unsigned int min_buffer_size, unsigned int max_buffer_size);

    // TODO: comment
    /**
     * Filters the new ball detection data, and returns the updated state of the ball
     * given the new data
     *
     * @param current_ball_state The current state of the Ball
     * @param new_ball_detections A list of new SSL Ball detections
     *
     * @return The updated state of the ball given the new data
     */
    std::optional<Ball> getFilteredData(const std::vector<SSLBallDetection>& new_ball_detections, const Field& field);

    boost::circular_buffer<SSLBallDetection> getResizedDetectionBuffer(
            boost::circular_buffer<SSLBallDetection> ball_detections);

//   private:
    // TODO: comments
    void addNewDetectionsToBuffer(std::vector<SSLBallDetection> new_ball_detections,
                                  const Field &field);
    Ball getLinearRegressionPositionAndVelocity(boost::circular_buffer<SSLBallDetection> ball_detections);
    BallVelocityEstimate estimateBallVelocity(boost::circular_buffer<SSLBallDetection> ball_detections);

    LinearRegressionResults getLinearRegressionLine(const boost::circular_buffer<SSLBallDetection> &ball_detections);


    // A circular buffer used to store previous ball detections, so we can use them
    // in the filter
    boost::circular_buffer<SSLBallDetection> ball_detection_buffer;
    unsigned int _min_buffer_size;
    unsigned int _max_buffer_size;
};

#pragma once

#include <vector>

#include "ai/world/ball.h"
#include "geom/point.h"
#include "util/timestamp.h"

/**
 * A lightweight datatype used to input new data into the filter.
 * We do this rather than taking the SSL_DetectionBall directly
 * so we can make this module more generic and abstract away
 * the protobuf for testing
 */
typedef struct
{
    Point position;
    double confidence;
    Timestamp timestamp;
} SSLBallDetection;

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
    explicit BallFilter() = default;

    /**
     * Returns the most up to date state of the Ball by filtering the new data
     * and using it to update the current state.
     *
     * @param new_ball_detections A list of new detection for the ball
     * @return The filtered and updated state of the ball
     */
    Ball getFilteredData(const Ball& current_ball_state,
                         const std::vector<SSLBallDetection>& new_ball_detections);
};

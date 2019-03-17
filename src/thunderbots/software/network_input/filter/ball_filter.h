#pragma once

#include <vector>

#include "ai/world/ball.h"
#include "geom/point.h"
#include "util/time/timestamp.h"

/**
 * A lightweight datatype used to input new data into the filter.
 * We do this rather than taking the SSL_DetectionBall directly
 * so we can make this module more generic and abstract away
 * the protobuf for testing
 */
struct SSLBallDetection
{
    Point position;
    double confidence;
    Timestamp timestamp;
};

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
     * Filters the new ball detection data, and returns the updated state of the ball
     * given the new data
     *
     * @param current_ball_state The current state of the Ball
     * @param new_ball_detections A list of new SSL Ball detections
     *
     * @return The updated state of the ball given the new data
     */
    Ball getFilteredData(const Ball& current_ball_state,
                         const std::vector<SSLBallDetection>& new_ball_detections);
};

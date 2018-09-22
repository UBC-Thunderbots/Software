#pragma once

#include <vector>

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
    AITimestamp timestamp;
} SSLBallData;

/**
 * A lightweight datatype used to pass filtered ball data. We use this rather than
 * something like std::pair so it's very explicit what data is being accessed, since
 * it's accessed by name rather than by index (first/second)
 */
typedef struct
{
    Point position;
    Vector velocity;
    AITimestamp timestamp;
} FilteredBallData;

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
    explicit BallFilter();

    /**
     * Updates the filter given a new set of data, and returns the most up to date
     * filtered data for the ball.
     *
     * @param new_ball_data A list of new datapoints for the ball
     * @return The filtered data for the ball
     */
    FilteredBallData getFilteredData(std::vector<SSLBallData> new_ball_data);
};

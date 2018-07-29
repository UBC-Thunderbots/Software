#pragma once

#include <vector>
#include "geom/point.h"

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
} SSLBallData;

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
     * Updates the filter given a new set of data
     */
    void update(std::vector<SSLBallData> new_ball_data);

    /**
     * Returns the filtered position of the ball
     *
     * @return the filtered position of the ball
     */
    Point getBallPosition();

    /**
     * Returns the filtered velocity of the ball
     *
     * @return the filtered velocity of the ball
     */
    Point getBallVelocity();

   private:
    Point current_ball_position;
    Point current_ball_velocity;
};

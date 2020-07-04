#pragma once

#include <optional>

#include "software/time/duration.h"
#include "software/world/ball_state.h"

/**
 * A BallModel predicts the future state of the ball
 */
class BallModel
{
   public:
    /**
     * Returns the estimated state of the ball at the specified amount of time in the
     * future
     *
     * @param time_in_future The Duration into the future at which to predict the
     * ball's position from when the BallModel was initialized
     *
     * @throws std::invalid_argument if the ball is estimating state at a time from the
     * past
     *
     * @return The future state of the ball
     */
    virtual BallState estimateFutureState(const Duration &time_in_future);

   private:
    /**
     * Estimates the future position of the ball
     *
     * @param initial_position The initial position of the ball
     * @param initial velocity The initial velocity of the ball in position units/s
     * @param acceleration The constant acceleration applied to the ball over time in
     * position units/s^2
     * @param time_in_future The time in the future to estimate position
     *
     * @return future position of the ball
     */
    Point estimateFuturePosition(const Point &initial_position,
                                 const Vector &intial_velocity,
                                 const Vector &acceleration, Duration time_in_future);
};

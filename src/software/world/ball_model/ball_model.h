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
     * @param duration_in_future The Duration into the future at which to predict the
     * ball's position from when the BallModel was initialized
     *
     * @throws std::invalid_argument if the ball is estimating state at a time from the
     * past
     *
     * @return The future state of the ball
     */
    virtual BallState estimateFutureState(const Duration &duration_in_future) = 0;

    virtual ~BallModel() = default;
};

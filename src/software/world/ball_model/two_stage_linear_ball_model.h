#pragma once

#include <optional>

#include "software/time/duration.h"
#include "software/world/ball_state.h"

/**
 * TwoStageLinearBallModel is a BallModel that makes the following assumptions about the
 * ball:
 * 1. no initial spin
 * 2. slides with a constant sliding friction above a certain speed threshold
 * 3. rolls with a constant rolling friction below a certain speed threshold
 * 4. the transition between sliding and rolling is instantaneous
 */
class TwoStageLinearBallModel
{
   public:
    TwoStageLinearBallModel() = delete;

    /**
     * Creates a new TwoStageLinearBallModel with the given friction
     *
     * @param initial_ball_state The initial state of the ball
     * @param rolling_friction_acceleration_m_per_s_squared The acceleration opposing the
     * direction of travel of a rolling ball
     * @param sliding_friction_acceleration_m_per_s_squared The acceleration opposing the
     * direction of travel of a sliding ball
     * @param sliding_to_rolling_speed_threshold_m_per_s The threshold above which the
     * ball slides and below which the ball rolls
     */
    explicit TwoStageLinearBallModel(
        BallState initial_ball_state,
        double rolling_friction_acceleration_m_per_s_squared = 0,
        double sliding_friction_acceleration_m_per_s_squared = 0,
        double sliding_to_rolling_speed_threshold_m_per_s    = 0);

    Vector estimateFutureVelocity(const Duration &duration_in_future);

   private:
    const BallState initial_ball_state_;
    // acceleration opposing the direction of travel of a rolling ball
    double rolling_friction_acceleration_m_per_s_squared_;
    // acceleration opposing the direction of travel of a sliding ball
    double sliding_friction_acceleration_m_per_s_squared_;
    // threshold above which the ball slides and below which the ball rolls
    double sliding_to_rolling_speed_threshold_m_per_s_;
};

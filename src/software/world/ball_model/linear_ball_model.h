#pragma once

#include <optional>

#include "software/world/ball_model/ball_model.h"

/**
 * LinearBallModel is a BallModel that makes the following assumptions about the ball:
 * 1. no initial spin
 * 2. slides with a constant sliding friction above a certain speed threshold
 * 3. rolls with a constant rolling friction below a certain speed threshold
 */
class LinearBallModel : public BallModel
{
   public:
    struct FrictionParameters
    {
        // acceleration opposing the direction of travel of a rolling ball
        double rolling_friction_acceleration;
        // acceleration opposing the direction of travel of a sliding ball
        double sliding_friction_acceleration;
        // threshold above which the ball slides and below which the ball rolls
        double rolling_sliding_speed_threshold;
    };

    LinearBallModel() = delete;

    /**
     * Creates a new LinearBallModel with the given friction
     *
     * @param initial_ball_state The initial state of the ball
     * @param friction_parameters The FrictionParameters for prediction
     */
    explicit LinearBallModel(
        BallState initial_ball_state,
        std::optional<FrictionParameters> friction_parameters = std::nullopt);

    BallState estimateFutureState(const Duration &time_in_future) override;

   private:
    BallState initial_ball_state;
    std::optional<FrictionParameters> friction_parameters;
};

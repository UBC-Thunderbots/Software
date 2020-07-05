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

    BallState estimateFutureState(double seconds_in_future) override;

   private:
    const BallState initial_ball_state_;
    const std::optional<FrictionParameters> friction_parameters_;

    /**
     * Applies linear friction model
     *
     * @param initial_ball_state initial ball state
     * @param seconds_in_future number of seconds into the future
     * @param friction_parameters defines friction for model
     *
     * @return future ball state
     */
    static BallState applyLinearFrictionModel(
        const BallState& initial_ball_state, double seconds_in_future,
        const FrictionParameters& friction_parameters);

    /**
     * Calculates the future ball state assuming constant acceleration opposing the
     * velocity of the ball
     *
     * @param initial_position initial_position of the ball
     * @param initial_velocity initial_velocity of the ball
     * @param friction_acceleration The magnitude of the acceleration due to friction
     * @param seconds_in_future number of seconds into the future
     *
     * @return future ball state
     */
    static BallState calculateFutureBallState(BallState initial_ball_state,
                                              double friction_acceleration,
                                              double seconds_in_future);

    /**
     * Returns absolute value of input friction parameter
     *
     * @param friction_parameters FrictionParameters to initialize
     *
     * @return friction parameter with no negative values
     */
    std::optional<FrictionParameters> initFrictionParameters(
        std::optional<FrictionParameters> fp);
};

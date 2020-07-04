#include "software/world/ball_model/linear_ball_model.h"

LinearBallModel::LinearBallModel(BallState initial_ball_state,
                                 std::optional<FrictionParameters> friction_parameters)
    : initial_ball_state(initial_ball_state), friction_parameters(friction_parameters)
{
}

BallState LinearBallModel::estimateFutureState(const Duration &time_in_future)
{
    double seconds_in_future = time_in_future.getSeconds();
    if (time_in_future < Duration::fromSeconds(0))
    {
        throw std::invalid_argument(
            "Position estimate is updating to a time in the past");
    }

    // Point future_position;
    // Vector future_velocity;
    if (friction_parameters)
    {
        return initial_ball_state;
    }
    else
    {
        return BallState(initial_ball_state.position() +
                             initial_ball_state.velocity() * seconds_in_future,
                         initial_ball_state.velocity());
    }
}

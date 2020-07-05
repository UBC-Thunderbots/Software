#include "software/world/ball_model/linear_ball_model.h"

#include "software/new_geom/util/physics.h"

LinearBallModel::LinearBallModel(BallState initial_ball_state,
                                 std::optional<FrictionParameters> friction_parameters)
    : initial_ball_state_(initial_ball_state), friction_parameters_(friction_parameters)
{
}

BallState LinearBallModel::estimateFutureState(double seconds_in_future)
{
    if (seconds_in_future < 0)
    {
        throw std::invalid_argument(
            "Position estimate is updating to a time in the past");
    }

    if (friction_parameters_)
    {
        return applyLinearFrictionModel(initial_ball_state_, seconds_in_future,
                                        *friction_parameters_);
    }
    else
    {
        Point future_position = initial_ball_state_.position() +
                                initial_ball_state_.velocity() * seconds_in_future;
        return BallState(future_position, initial_ball_state_.velocity());
    }
}

BallState LinearBallModel::applyLinearFrictionModel(
    const BallState &initial_ball_state, double seconds_in_future,
    const FrictionParameters &friction_parameters)
{
    double initial_velocity_length = initial_ball_state.velocity().length();
    if (initial_velocity_length > friction_parameters.rolling_sliding_speed_threshold)
    {
        double seconds_until_rolling =
            initial_velocity_length / friction_parameters.sliding_friction_acceleration;
        if (seconds_in_future <= seconds_until_rolling)
        {
            return calculateFutureBallState(
                initial_ball_state, friction_parameters.sliding_friction_acceleration,
                seconds_in_future);
        }
        else
        {
            BallState initial_rolling_state = calculateFutureBallState(
                initial_ball_state, friction_parameters.sliding_friction_acceleration,
                seconds_until_rolling);
            return calculateFutureBallState(
                initial_rolling_state, friction_parameters.rolling_friction_acceleration,
                seconds_in_future - seconds_until_rolling);
        }
    }
    else
    {
        return calculateFutureBallState(initial_ball_state,
                                        friction_parameters.rolling_friction_acceleration,
                                        seconds_in_future);
    }
}

BallState LinearBallModel::calculateFutureBallState(BallState initial_ball_state,
                                                    double friction_acceleration,
                                                    double seconds_in_future)
{
    Vector acceleration_vector =
        initial_ball_state.velocity().normalize(-friction_acceleration);
    Point future_position = calculateFuturePosition(
        initial_ball_state.position(), initial_ball_state.velocity(), acceleration_vector,
        seconds_in_future);
    Vector future_velocity = calculateFutureVelocity(
        initial_ball_state.velocity(), acceleration_vector, seconds_in_future);

    return BallState(future_position, future_velocity);
}

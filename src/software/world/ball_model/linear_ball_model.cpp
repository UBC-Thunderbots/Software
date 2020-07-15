#include "software/world/ball_model/linear_ball_model.h"

#include "software/physics/physics.h"

LinearBallModel::LinearBallModel(BallState initial_ball_state,
                                 std::optional<FrictionParameters> friction_parameters)
    : initial_ball_state_(initial_ball_state),
      friction_parameters_(initFrictionParameters(friction_parameters))
{
}

BallState LinearBallModel::estimateFutureState(Duration duration_in_future)
{
    if (duration_in_future < Duration::fromSeconds(0.0))
    {
        throw std::invalid_argument(
            "Position estimate is updating to a time in the past");
    }

    if (friction_parameters_)
    {
        return applyLinearFrictionModel(initial_ball_state_, duration_in_future,
                                        *friction_parameters_);
    }
    else
    {
        Point future_position =
            initial_ball_state_.position() +
            initial_ball_state_.velocity() * duration_in_future.getSeconds();
        return BallState(future_position, initial_ball_state_.velocity());
    }
}

BallState LinearBallModel::applyLinearFrictionModel(
    const BallState &initial_ball_state, Duration duration_in_future,
    const FrictionParameters &friction_parameters)
{
    double initial_velocity_length = initial_ball_state.velocity().length();
    if (initial_velocity_length > friction_parameters.rolling_sliding_speed_threshold)
    {
        Duration duration_until_rolling =
            Duration::fromSeconds((initial_velocity_length -
                                   friction_parameters.rolling_sliding_speed_threshold) /
                                  friction_parameters.sliding_friction_acceleration);
        if (duration_in_future <= duration_until_rolling)
        {
            return calculateFutureBallState(
                initial_ball_state, friction_parameters.sliding_friction_acceleration,
                duration_in_future);
        }
        else
        {
            BallState initial_rolling_state = calculateFutureBallState(
                initial_ball_state, friction_parameters.sliding_friction_acceleration,
                duration_until_rolling);
            Duration duration_to_spend_rolling =
                duration_in_future - duration_until_rolling;
            Duration duration_until_stopped =
                Duration::fromSeconds(initial_rolling_state.velocity().length() /
                                      friction_parameters.rolling_friction_acceleration);
            if (duration_to_spend_rolling <= duration_until_stopped)
            {
                return calculateFutureBallState(
                    initial_rolling_state,
                    friction_parameters.rolling_friction_acceleration,
                    duration_to_spend_rolling);
            }
            else
            {
                return calculateFutureBallState(
                    initial_rolling_state,
                    friction_parameters.rolling_friction_acceleration,
                    duration_until_stopped);
            }
        }
    }
    else
    {
        return calculateFutureBallState(initial_ball_state,
                                        friction_parameters.rolling_friction_acceleration,
                                        duration_in_future);
    }
}

BallState LinearBallModel::calculateFutureBallState(BallState initial_ball_state,
                                                    double friction_acceleration,
                                                    Duration duration_in_future)
{
    Vector acceleration_vector =
        initial_ball_state.velocity().normalize(-friction_acceleration);
    Point future_position = calculateFuturePosition(
        initial_ball_state.position(), initial_ball_state.velocity(), acceleration_vector,
        duration_in_future.getSeconds());
    Vector future_velocity =
        calculateFutureVelocity(initial_ball_state.velocity(), acceleration_vector,
                                duration_in_future.getSeconds());

    return BallState(future_position, future_velocity);
}

std::optional<LinearBallModel::FrictionParameters>
LinearBallModel::initFrictionParameters(std::optional<FrictionParameters> fp)
{
    if (fp)
    {
        return FrictionParameters{
            .rolling_friction_acceleration = std::abs(fp->rolling_friction_acceleration),
            .sliding_friction_acceleration = std::abs(fp->sliding_friction_acceleration),
            .rolling_sliding_speed_threshold =
                std::abs(fp->rolling_sliding_speed_threshold)};
    }
    else
    {
        return std::nullopt;
    }
}

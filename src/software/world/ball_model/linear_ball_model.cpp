#include "software/world/ball_model/linear_ball_model.h"

#include "software/physics/physics.h"

LinearBallModel::LinearBallModel(
    BallState initial_ball_state, double rolling_friction_acceleration_m_per_s_squared,
    double sliding_friction_acceleration_m_per_s_squared,
    double sliding_to_rolling_transition_speed_threshold_m_per_s)
    : initial_ball_state_(initial_ball_state),
      rolling_friction_acceleration_m_per_s_squared_(
          rolling_friction_acceleration_m_per_s_squared),
      sliding_friction_acceleration_m_per_s_squared_(
          sliding_friction_acceleration_m_per_s_squared),
      sliding_to_rolling_transition_speed_threshold_m_per_s_(
          sliding_to_rolling_transition_speed_threshold_m_per_s)
{
    if (rolling_friction_acceleration_m_per_s_squared_ < 0 ||
        sliding_friction_acceleration_m_per_s_squared_ < 0 ||
        sliding_to_rolling_transition_speed_threshold_m_per_s_ < 0)
    {
        throw std::invalid_argument("All friction parameters must be positive");
    }
}

BallState LinearBallModel::estimateFutureState(const Duration &duration_in_future)
{
    if (duration_in_future < Duration::fromSeconds(0.0))
    {
        throw std::invalid_argument(
            "Position estimate is updating to a time in the past");
    }

    return applyLinearFrictionModel(duration_in_future);
}

BallState LinearBallModel::applyLinearFrictionModel(
    const Duration &duration_in_future) const
{
    double initial_velocity_length = initial_ball_state_.velocity().length();
    if (initial_velocity_length > sliding_to_rolling_transition_speed_threshold_m_per_s_)
    {
        Duration duration_until_rolling = Duration::fromSeconds(
            (initial_velocity_length -
             sliding_to_rolling_transition_speed_threshold_m_per_s_) /
            sliding_friction_acceleration_m_per_s_squared_);
        if (duration_in_future <= duration_until_rolling)
        {
            return calculateFutureBallState(
                initial_ball_state_, sliding_friction_acceleration_m_per_s_squared_,
                duration_in_future);
        }
        else
        {
            BallState initial_rolling_state = calculateFutureBallState(
                initial_ball_state_, sliding_friction_acceleration_m_per_s_squared_,
                duration_until_rolling);
            Duration duration_to_spend_rolling =
                duration_in_future - duration_until_rolling;
            Duration duration_until_stopped =
                Duration::fromSeconds(initial_rolling_state.velocity().length() /
                                      rolling_friction_acceleration_m_per_s_squared_);
            if (duration_to_spend_rolling <= duration_until_stopped)
            {
                return calculateFutureBallState(
                    initial_rolling_state, rolling_friction_acceleration_m_per_s_squared_,
                    duration_to_spend_rolling);
            }
            else
            {
                return calculateFutureBallState(
                    initial_rolling_state, rolling_friction_acceleration_m_per_s_squared_,
                    duration_until_stopped);
            }
        }
    }
    else
    {
        return calculateFutureBallState(initial_ball_state_,
                                        rolling_friction_acceleration_m_per_s_squared_,
                                        duration_in_future);
    }
}

BallState LinearBallModel::calculateFutureBallState(BallState initial_ball_state_,
                                                    double friction_acceleration,
                                                    Duration duration_in_future) const
{
    const Vector acceleration_vector =
        initial_ball_state_.velocity().normalize(-friction_acceleration);
    const Point future_position = calculateFuturePosition(
        initial_ball_state_.position(), initial_ball_state_.velocity(),
        acceleration_vector, duration_in_future.getSeconds());
    const Vector future_velocity =
        calculateFutureVelocity(initial_ball_state_.velocity(), acceleration_vector,
                                duration_in_future.getSeconds());

    return BallState(future_position, future_velocity);
}

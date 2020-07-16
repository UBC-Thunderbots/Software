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
        sliding_to_rolling_transition_speed_threshold_m_per_s < 0)
    {
        throw std::invalid_argument("All friction parameters must be positive");
    }
}

BallState LinearBallModel::estimateFutureState(Duration duration_in_future)
{
    if (duration_in_future < Duration::fromSeconds(0.0))
    {
        throw std::invalid_argument(
            "Position estimate is updating to a time in the past");
    }

    return applyLinearFrictionModel(
        initial_ball_state_, duration_in_future,
        rolling_friction_acceleration_m_per_s_squared_,
        sliding_friction_acceleration_m_per_s_squared_,
        sliding_to_rolling_transition_speed_threshold_m_per_s_);
}

BallState LinearBallModel::applyLinearFrictionModel(
    const BallState& initial_ball_state, Duration duration_in_future,
    double rolling_friction_acceleration_m_per_s_squared,
    double sliding_friction_acceleration_m_per_s_squared,
    double sliding_to_rolling_transition_speed_threshold_m_per_s)
{
    double initial_velocity_length = initial_ball_state.velocity().length();
    if (initial_velocity_length > sliding_to_rolling_transition_speed_threshold_m_per_s)
    {
        Duration duration_until_rolling = Duration::fromSeconds(
            (initial_velocity_length -
             sliding_to_rolling_transition_speed_threshold_m_per_s) /
            sliding_friction_acceleration_m_per_s_squared);
        if (duration_in_future <= duration_until_rolling)
        {
            return calculateFutureBallState(initial_ball_state,
                                            sliding_friction_acceleration_m_per_s_squared,
                                            duration_in_future);
        }
        else
        {
            BallState initial_rolling_state = calculateFutureBallState(
                initial_ball_state, sliding_friction_acceleration_m_per_s_squared,
                duration_until_rolling);
            Duration duration_to_spend_rolling =
                duration_in_future - duration_until_rolling;
            Duration duration_until_stopped =
                Duration::fromSeconds(initial_rolling_state.velocity().length() /
                                      rolling_friction_acceleration_m_per_s_squared);
            if (duration_to_spend_rolling <= duration_until_stopped)
            {
                return calculateFutureBallState(
                    initial_rolling_state, rolling_friction_acceleration_m_per_s_squared,
                    duration_to_spend_rolling);
            }
            else
            {
                return calculateFutureBallState(
                    initial_rolling_state, rolling_friction_acceleration_m_per_s_squared,
                    duration_until_stopped);
            }
        }
    }
    else
    {
        return calculateFutureBallState(initial_ball_state,
                                        rolling_friction_acceleration_m_per_s_squared,
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

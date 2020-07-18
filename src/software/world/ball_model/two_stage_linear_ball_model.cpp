#include "software/world/ball_model/two_stage_linear_ball_model.h"

#include "software/physics/physics.h"

TwoStageLinearBallModel::TwoStageLinearBallModel(
    BallState initial_ball_state, double rolling_friction_acceleration_m_per_s_squared,
    double sliding_friction_acceleration_m_per_s_squared,
    double sliding_to_rolling_speed_threshold_m_per_s)
    : initial_ball_state_(initial_ball_state),
      rolling_friction_acceleration_m_per_s_squared_(
          rolling_friction_acceleration_m_per_s_squared),
      sliding_friction_acceleration_m_per_s_squared_(
          sliding_friction_acceleration_m_per_s_squared),
      sliding_to_rolling_speed_threshold_m_per_s_(
          sliding_to_rolling_speed_threshold_m_per_s)
{
    if (rolling_friction_acceleration_m_per_s_squared_ < 0 ||
        sliding_friction_acceleration_m_per_s_squared_ < 0 ||
        sliding_to_rolling_speed_threshold_m_per_s_ < 0)
    {
        throw std::invalid_argument("All friction parameters must be positive");
    }
}

BallState TwoStageLinearBallModel::estimateFutureState(const Duration &duration_in_future)
{
    if (duration_in_future < Duration::fromSeconds(0.0))
    {
        throw std::invalid_argument(
            "Position estimate is updating to a time in the past");
    }

    return applyLinearFrictionModel(duration_in_future);
}

BallState TwoStageLinearBallModel::applyLinearFrictionModel(
    const Duration &duration_in_future) const
{
    const double seconds_in_future     = duration_in_future.getSeconds();
    const double initial_speed_m_per_s = initial_ball_state_.velocity().length();

    // Figure out how long the ball will roll/slide, if at all
    const double max_sliding_duration_secs =
        (initial_speed_m_per_s - sliding_to_rolling_speed_threshold_m_per_s_) /
        sliding_friction_acceleration_m_per_s_squared_;
    const double sliding_duration_secs =
        std::max(0.0, std::min(seconds_in_future, max_sliding_duration_secs));
    const double max_rolling_duration_secs =
        std::min(initial_speed_m_per_s, sliding_to_rolling_speed_threshold_m_per_s_) /
        rolling_friction_acceleration_m_per_s_squared_;
    const double rolling_duration_secs = std::max(
        0.0,
        std::min(seconds_in_future - sliding_duration_secs, max_rolling_duration_secs));

    // Figure out where the ball is after the two stages
    const BallState ball_state_after_sliding = calculateFutureBallState(
        initial_ball_state_, sliding_friction_acceleration_m_per_s_squared_,
        Duration::fromSeconds(sliding_duration_secs));
    const BallState ball_state_after_rolling = calculateFutureBallState(
        ball_state_after_sliding, rolling_friction_acceleration_m_per_s_squared_,
        Duration::fromSeconds(rolling_duration_secs));

    return ball_state_after_rolling;
}

BallState TwoStageLinearBallModel::calculateFutureBallState(
    const BallState &initial_ball_state,
    const double constant_friction_acceleration_m_per_s,
    const Duration &duration_in_future) const
{
    const Vector acceleration_vector =
        initial_ball_state.velocity().normalize(-constant_friction_acceleration_m_per_s);
    const Point future_position = calculateFuturePosition(
        initial_ball_state.position(), initial_ball_state.velocity(), acceleration_vector,
        duration_in_future);
    const Vector future_velocity = calculateFutureVelocity(
        initial_ball_state.velocity(), acceleration_vector, duration_in_future);

    return BallState(future_position, future_velocity);
}

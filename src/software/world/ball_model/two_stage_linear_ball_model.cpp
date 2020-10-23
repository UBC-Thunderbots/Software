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

Vector TwoStageLinearBallModel::estimateFutureVelocity(const Duration &duration_in_future)
{
    const double seconds_in_future     = duration_in_future.toSeconds();
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
    const Vector sliding_acceleration_vector = initial_ball_state_.velocity().normalize(
        -sliding_friction_acceleration_m_per_s_squared_);
    const Vector velocity_after_sliding = calculateFutureVelocity(
        initial_ball_state_.velocity(), sliding_acceleration_vector,
        Duration::fromSeconds(sliding_duration_secs));

    const Vector rolling_acceleration_vector = initial_ball_state_.velocity().normalize(
        -rolling_friction_acceleration_m_per_s_squared_);
    const Vector velocity_after_rolling =
        calculateFutureVelocity(velocity_after_sliding, rolling_acceleration_vector,
                                Duration::fromSeconds(rolling_duration_secs));

    return velocity_after_rolling;
}

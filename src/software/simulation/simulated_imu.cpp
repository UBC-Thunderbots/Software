#include "software/simulation/simulated_imu.h"

SimulatedImuService::SimulatedImuService(const NoiseConfig& noise_config,
                                         unsigned int seed)
    : noise_config_(noise_config),
      prev_angular_velocity_(std::nullopt),
      prev_velocity_(std::nullopt),
      random_generator_(seed),
      standard_normal_distribution_(0.0, 1.0)
{
}

void SimulatedImuService::poll(const RobotState& true_state,
                               TbotsProto::RobotStatus& robot_status,
                               const Duration& time_elapsed_since_last_poll)
{
    const double time_elapsed_since_last_poll_s =
        time_elapsed_since_last_poll.toSeconds();

    const AngularVelocity angular_velocity = readAngularVelocity(true_state);

    TbotsProto::ImuStatus imu_status;
    imu_status.mutable_angular_velocity()->set_radians_per_second(
        angular_velocity.toRadians());

    if (prev_angular_velocity_.has_value() && prev_velocity_.has_value() &&
        time_elapsed_since_last_poll_s > 0)
    {
        const double angular_acceleration_rad_per_s_2 =
            (angular_velocity.toRadians() - prev_angular_velocity_->toRadians()) /
            time_elapsed_since_last_poll_s;
        imu_status.mutable_angular_acceleration()->set_radians_per_second_squared(
            angular_acceleration_rad_per_s_2);

        const Vector linear_acceleration = readLinearAcceleration(
            true_state, prev_velocity_.value(), time_elapsed_since_last_poll_s);
        imu_status.mutable_linear_acceleration()->set_x_component_meters(
            linear_acceleration.x());
        imu_status.mutable_linear_acceleration()->set_y_component_meters(
            linear_acceleration.y());
    }

    prev_angular_velocity_ = angular_velocity;
    prev_velocity_         = true_state.velocity();

    *(robot_status.mutable_imu_status()) = imu_status;
}

void SimulatedImuService::reset()
{
    prev_angular_velocity_.reset();
    prev_velocity_.reset();
}

AngularVelocity SimulatedImuService::readAngularVelocity(const RobotState& true_state)
{
    const double noise_rad_per_s = noise_config_.angular_velocity_stddev_rad_per_s *
                                   standard_normal_distribution_(random_generator_);

    return AngularVelocity::fromRadians(true_state.angularVelocity().toRadians() +
                                        noise_config_.angular_velocity_bias_rad_per_s +
                                        noise_rad_per_s);
}

Vector SimulatedImuService::readLinearAcceleration(
    const RobotState& true_state, const Vector& previous_velocity,
    const double time_elapsed_since_last_poll_s)
{
    const Vector global_acceleration =
        (true_state.velocity() - previous_velocity) / time_elapsed_since_last_poll_s;

    const Vector local_acceleration =
        global_acceleration.rotate(-true_state.orientation());

    const double noise_x_m_per_s_2 = noise_config_.linear_acceleration_stddev_m_per_s_2 *
                                     standard_normal_distribution_(random_generator_);
    const double noise_y_m_per_s_2 = noise_config_.linear_acceleration_stddev_m_per_s_2 *
                                     standard_normal_distribution_(random_generator_);

    return local_acceleration + noise_config_.linear_acceleration_bias_m_per_s_2 +
           Vector(noise_x_m_per_s_2, noise_y_m_per_s_2);
}

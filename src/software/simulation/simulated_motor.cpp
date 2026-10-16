#include "software/simulation/simulated_motor.h"


SimulatedMotorService::SimulatedMotorService(
    const robot_constants::RobotConstants& robot_constants,
    const NoiseConfig& noise_config, unsigned int seed)
    : noise_config_(noise_config),
      euclidean_to_four_wheel_(robot_constants),
      target_wheel_velocities_(std::nullopt),
      random_generator_(seed),
      standard_normal_distribution_(0.0, 1.0)
{
}

TbotsProto::DirectControlPrimitive SimulatedMotorService::poll(
    const TbotsProto::DirectControlPrimitive& primitive, const RobotState& true_state,
    TbotsProto::RobotStatus& robot_status, const Duration& time_elapsed_since_last_poll)
{
    const WheelSpace_t current_wheel_velocities = readWheelVelocities(true_state);

    if (!target_wheel_velocities_.has_value())
    {
        target_wheel_velocities_ = current_wheel_velocities;
    }

    updateTargetVelocities(primitive, time_elapsed_since_last_poll.toSeconds());

    updateMotorStatus(robot_status, current_wheel_velocities);

    const EuclideanSpace_t ramped_euclidean_velocity =
        euclidean_to_four_wheel_.getEuclideanVelocity(target_wheel_velocities_.value());

    // Copy the primitive so that the dribbler and kick/chip commands pass through
    TbotsProto::DirectControlPrimitive ramped_primitive = primitive;
    auto* direct_velocity =
        ramped_primitive.mutable_motor_control()->mutable_direct_velocity_control();
    direct_velocity->mutable_velocity()->set_x_component_meters(
        ramped_euclidean_velocity[0]);
    direct_velocity->mutable_velocity()->set_y_component_meters(
        ramped_euclidean_velocity[1]);
    direct_velocity->mutable_angular_velocity()->set_radians_per_second(
        ramped_euclidean_velocity[2]);

    return ramped_primitive;
}

WheelSpace_t SimulatedMotorService::readWheelVelocities(const RobotState& true_state)
{
    const Vector local_velocity   = true_state.localVelocity();
    WheelSpace_t wheel_velocities = euclidean_to_four_wheel_.getWheelVelocity(
        {local_velocity.x(), local_velocity.y(),
         true_state.angularVelocity().toRadians()});

    for (Eigen::Index i = 0; i < wheel_velocities.size(); i++)
    {
        wheel_velocities[i] += noise_config_.wheel_velocity_stddev_m_per_s *
                               standard_normal_distribution_(random_generator_);
    }

    return wheel_velocities;
}

void SimulatedMotorService::updateTargetVelocities(
    const TbotsProto::DirectControlPrimitive& primitive,
    const double time_elapsed_since_last_poll_s)
{
    // An unset direct velocity control reads as all zeros, so a primitive without
    // one ramps the robot to a stop
    const TbotsProto::MotorControl::DirectVelocityControl& direct_velocity =
        primitive.motor_control().direct_velocity_control();

    const EuclideanSpace_t target_euclidean_velocity = {
        direct_velocity.velocity().x_component_meters(),
        direct_velocity.velocity().y_component_meters(),
        direct_velocity.angular_velocity().radians_per_second()};

    target_wheel_velocities_ = euclidean_to_four_wheel_.rampWheelVelocity(
        target_wheel_velocities_.value(),
        euclidean_to_four_wheel_.getWheelVelocity(target_euclidean_velocity),
        time_elapsed_since_last_poll_s);
}

void SimulatedMotorService::updateMotorStatus(
    TbotsProto::RobotStatus& robot_status,
    const WheelSpace_t& current_wheel_velocities) const
{
    TbotsProto::MotorStatus motor_status;

    motor_status.mutable_front_left()->set_wheel_velocity(
        static_cast<float>(current_wheel_velocities[FRONT_LEFT_WHEEL_SPACE_INDEX]));
    motor_status.mutable_front_right()->set_wheel_velocity(
        static_cast<float>(current_wheel_velocities[FRONT_RIGHT_WHEEL_SPACE_INDEX]));
    motor_status.mutable_back_left()->set_wheel_velocity(
        static_cast<float>(current_wheel_velocities[BACK_LEFT_WHEEL_SPACE_INDEX]));
    motor_status.mutable_back_right()->set_wheel_velocity(
        static_cast<float>(current_wheel_velocities[BACK_RIGHT_WHEEL_SPACE_INDEX]));

    const EuclideanSpace_t current_euclidean_velocity =
        euclidean_to_four_wheel_.getEuclideanVelocity(current_wheel_velocities);

    motor_status.mutable_local_velocity()->set_x_component_meters(
        current_euclidean_velocity[0]);
    motor_status.mutable_local_velocity()->set_y_component_meters(
        current_euclidean_velocity[1]);
    motor_status.mutable_angular_velocity()->set_radians_per_second(
        current_euclidean_velocity[2]);

    const EuclideanSpace_t target_euclidean_velocity =
        euclidean_to_four_wheel_.getEuclideanVelocity(target_wheel_velocities_.value());

    motor_status.mutable_target_local_velocity()->set_x_component_meters(
        target_euclidean_velocity[0]);
    motor_status.mutable_target_local_velocity()->set_y_component_meters(
        target_euclidean_velocity[1]);
    motor_status.mutable_target_angular_velocity()->set_radians_per_second(
        target_euclidean_velocity[2]);

    *(robot_status.mutable_motor_status()) = motor_status;
}

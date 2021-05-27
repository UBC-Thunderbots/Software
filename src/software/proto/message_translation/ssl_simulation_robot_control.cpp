#include "software/proto/message_translation/ssl_simulation_robot_control.h"

#include "shared/constants.h"

extern "C"
{
#include "firmware/shared/physics.h"
}

// Converts rpm and wheel_radius [m] to speed [m/s]
float rpm_to_m_per_s(float rpm, float wheel_radius)
{
    return (2 * (float)M_PI * rpm * wheel_radius) / 60.0f;
}

std::unique_ptr<SSLSimulationProto::RobotMoveCommand> createRobotMoveCommand(
    double wheel_rpm_front_right, double wheel_rpm_front_left, double wheel_rpm_back_left,
    double wheel_rpm_back_right, float front_wheel_angle_deg, float back_wheel_angle_deg,
    float wheel_radius)
{
    auto move_local_velocity = SSLSimulationProto::MoveLocalVelocity();

    // Convert the units of wheel speeds to m/s
    float front_left_m_per_s =
        rpm_to_m_per_s(static_cast<float>(wheel_rpm_front_left), wheel_radius);
    float back_left_m_per_s =
        rpm_to_m_per_s(static_cast<float>(wheel_rpm_back_left), wheel_radius);
    float back_right_m_per_s =
        rpm_to_m_per_s(static_cast<float>(wheel_rpm_back_right), wheel_radius);
    float front_right_m_per_s =
        rpm_to_m_per_s(static_cast<float>(wheel_rpm_front_right), wheel_radius);
    float wheel_speeds[4]{front_left_m_per_s, back_left_m_per_s, back_right_m_per_s,
                          front_right_m_per_s};
    float robot_local_speed[3]{0.0, 0.0, 0.0};
    shared_physics_speed4ToSpeed3(wheel_speeds, robot_local_speed, front_wheel_angle_deg,
                                  back_wheel_angle_deg);

    // Convert speed [m/s] to angular velocity [rad/s]
    robot_local_speed[2] =
        robot_local_speed[2] / static_cast<float>(ROBOT_MAX_RADIUS_METERS);

    move_local_velocity.set_forward(robot_local_speed[0]);
    move_local_velocity.set_left(robot_local_speed[1]);
    move_local_velocity.set_angular(robot_local_speed[2]);

    auto move_command = std::make_unique<SSLSimulationProto::RobotMoveCommand>();

    *(move_command->mutable_local_velocity()) = move_local_velocity;

    return move_command;
}

std::unique_ptr<SSLSimulationProto::RobotCommand> createRobotCommand(
    unsigned robot_id, std::unique_ptr<SSLSimulationProto::RobotMoveCommand> move_command,
    std::optional<double> kick_speed, std::optional<double> kick_angle,
    std::optional<double> dribbler_speed)
{
    auto robot_command = std::make_unique<SSLSimulationProto::RobotCommand>();

    robot_command->set_id(robot_id);

    if (kick_speed.has_value())
    {
        robot_command->set_kick_speed(static_cast<float>(kick_speed.value()));
    }
    if (kick_angle.has_value())
    {
        robot_command->set_kick_angle(static_cast<float>(kick_angle.value()));
    }
    if (dribbler_speed.has_value())
    {
        robot_command->set_dribbler_speed(static_cast<float>(dribbler_speed.value()));
    }

    *(robot_command->mutable_move_command()) = *move_command;

    return robot_command;
}

std::unique_ptr<SSLSimulationProto::RobotControl> createRobotControl(
    std::vector<std::unique_ptr<SSLSimulationProto::RobotCommand>> robot_commands)
{
    auto robot_control = std::make_unique<SSLSimulationProto::RobotControl>();

    for (auto&& robot_command : robot_commands)
    {
        *(robot_control->add_robot_commands()) = *robot_command;
    }

    return robot_control;
}

#include "software/proto/message_translation/ssl_simulation_robot_control.h"

std::unique_ptr<SSLSimulationProto::MoveWheelVelocity> createMoveWheelVelocity(
    double front_right, double front_left, double back_left, double back_right)
{
    auto move_local_velocity = std::make_unique<SSLSimulationProto::MoveWheelVelocity>();

    move_local_velocity->set_front_left(static_cast<float>(front_left));
    move_local_velocity->set_front_right(static_cast<float>(front_right));
    move_local_velocity->set_back_left(static_cast<float>(back_left));
    move_local_velocity->set_back_right(static_cast<float>(back_right));

    return move_local_velocity;
}

std::unique_ptr<SSLSimulationProto::RobotMoveCommand> createRobotMoveCommand(
    std::unique_ptr<SSLSimulationProto::MoveWheelVelocity> move_wheel_velocity)
{
    auto move_command = std::make_unique<SSLSimulationProto::RobotMoveCommand>();

    *(move_command->mutable_wheel_velocity()) = *move_wheel_velocity;

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

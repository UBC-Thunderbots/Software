#pragma once

#include "software/proto/ssl_simulation_robot_control.pb.h"

/**
 * Creates a MoveWheelVelocity proto
 *
 * @param team_colour the team colour to get game state for
 * @param front_right Velocity [m/s] of front right wheel
 * @param front_left Velocity [m/s] of front left wheel
 * @param back_right Velocity [m/s] of back right wheel
 * @param back_left Velocity [m/s] of back left wheel
 *
 * @return MoveWheelVelocity proto
 */
std::unique_ptr<SSLSimulationProto::MoveWheelVelocity> createMoveWheelVelocity(
    double front_right, double front_left, double back_left, double back_right);


/**
 * Creates a RobotMoveCommand proto
 *
 * @param move_wheel_velocity The move wheel velocity proto
 *
 * @return RobotMoveCommand proto
 */
std::unique_ptr<SSLSimulationProto::RobotMoveCommand> createRobotMoveCommand(
    std::unique_ptr<SSLSimulationProto::MoveWheelVelocity> move_wheel_velocity);

/**
 * Creates a RobotCommand proto
 *
 * @param robot_id The id this RobotCommand is for
 * @param move_command The move command for this robot
 * @param kick_speed The speed to kick at [m/s]
 * @param kick_angle The angle to chip at [r]
 * @param dribbler_speed The speed to dribble at [rpm]
 *
 * @return RobotCommand proto
 */
std::unique_ptr<SSLSimulationProto::RobotCommand> createRobotCommand(
    unsigned robot_id, std::unique_ptr<SSLSimulationProto::RobotMoveCommand> move_command,
    double kick_speed, double kick_angle, double dribbler_speed);

/**
 * Creates a RobotControl proto
 *
 * @param robot_commands A vector of robot commands
 *
 * @return RobotControl proto
 */
std::unique_ptr<SSLSimulationProto::RobotControl> createRobotControl(
    std::vector<std::unique_ptr<SSLSimulationProto::RobotCommand>> robot_commands);

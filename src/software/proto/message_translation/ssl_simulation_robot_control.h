#pragma once

#include "software/proto/ssl_simulation_robot_control.pb.h"
#include "src/amun/simulator/simulator.h"
#include "software/proto/ssl_simulation_robot_feedback.pb.h"

/**
 * Creates a RobotMoveCommand proto
 *
 * @param wheel_rpm_front_right Speed [rpm] of front right wheel
 * @param wheel_rpm_front_left Speed [rpm] of front left wheel
 * @param wheel_rpm_back_right Speed [rpm] of back right wheel
 * @param wheel_rpm_back_left Speed [rpm] of back left wheel
 *
 * @return RobotMoveCommand proto
 */
std::unique_ptr<sslsim::RobotMoveCommand> createRobotMoveCommand(
    double wheel_rpm_front_right, double wheel_rpm_front_left, double wheel_rpm_back_left,
    double wheel_rpm_back_right);

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
std::unique_ptr<sslsim::RobotCommand> createRobotCommand(
    unsigned robot_id, std::unique_ptr<sslsim::RobotMoveCommand> move_command,
    std::optional<double> kick_speed, std::optional<double> kick_angle,
    std::optional<double> dribbler_speed);

/**
 * Creates a RobotControl proto
 *
 * @param robot_commands A vector of robot commands
 *
 * @return RobotControl proto
 */
std::unique_ptr<sslsim::RobotControl> createRobotControl(
    std::vector<std::unique_ptr<sslsim::RobotCommand>> robot_commands);

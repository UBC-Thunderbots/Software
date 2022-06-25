#pragma once

#include <optional>

#include "proto/ssl_simulation_robot_control.pb.h"
#include "proto/ssl_simulation_robot_feedback.pb.h"
#include "proto/tbots_software_msgs.pb.h"
#include "shared/robot_constants.h"

/**
 * Creates a RobotMoveCommand proto
 *
 * @param direct_control The DirectControlPrimitive to create from
 * @param front_wheel_angle_deg angle between each front wheel and the y axis of the robot
 * in degrees
 * @param back_wheel_angle_deg angle between each back wheel and the y axis of the robot
 * in degrees
 * @param wheel_radius_meters The radius of the wheel in meters
 *
 * @return RobotMoveCommand proto
 */
std::unique_ptr<SSLSimulationProto::RobotMoveCommand> createRobotMoveCommand(
    const TbotsProto::DirectControlPrimitive& direct_control, float front_wheel_angle_deg,
    float back_wheel_angle_deg, float wheel_radius_meters);

/**
 * Creates a RobotCommand proto from Direct Control Primitive
 *
 * @param robot_id The id this RobotCommand is for
 * @param direct_control The Direct Control Primitive to create this RobotCommand from
 * @param robot_constants The Robot Constant to create this RobotCommand from
 * @return
 */
std::unique_ptr<SSLSimulationProto::RobotCommand> getRobotCommandFromDirectControl(
    unsigned int robot_id,
    std::unique_ptr<TbotsProto::DirectControlPrimitive> direct_control,
    RobotConstants_t& robot_constants);

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
    unsigned int robot_id,
    std::unique_ptr<SSLSimulationProto::RobotMoveCommand> move_command,
    std::optional<double> kick_speed, std::optional<double> kick_angle,
    std::optional<double> dribbler_speed);

/**
 * Creates a RobotControl proto
 *
 * @param robot_commands A vector of robot commands
 *
 * @return RobotControl proto
 */
std::unique_ptr<SSLSimulationProto::RobotControl> createRobotControl(
    std::vector<std::unique_ptr<SSLSimulationProto::RobotCommand>> robot_commands);

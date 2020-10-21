#pragma once
#include "shared/proto/robot_log_msg.nanopb.h"

/**
 * Initializes the logger with the given robot id and the robot log proto handler
 *
 * @param robot_id The robot id the logs are coming from
 * @param handle_robot_log_msg The handler for Robot Log protos 
 */
void app_logger_init(uint8_t robot_id, void (*handle_robot_log_msg)(TbotsProto_RobotLog));

/**
 * Log!
 *
 * @param TbotsProto_RobotLog the log msg
 */
void app_logger_log(TbotsProto_RobotLog log_msg);

#pragma once

#include <memory>
#include "shared/proto/robot_log_msg.pb.h"
#include <g3log/logmessage.hpp>


/**
 * g3log message to protobuf conversions
 *
 * @param msg the log message
 * @param robot_id id of the robot sending this message
 * @param level the log level proto of the log message
 * @param file the file where the log was produced
 * @param line_number the line number where the log was produced
 *
 *
 * @return The unique_ptr to the converted RobotLog proto
 */

std::unique_ptr<TbotsProto::RobotLog> createRobotLog(g3::LogMessageMover log_entry, int robot_id);


/**
 * log level to protobuf msg conversions
 *
 * @param level the log level of the log message
 *
 * @return The unique_ptr to the converted LogLevel proto
 */
std::unique_ptr<TbotsProto::LogLevel> createLogLevel(std::string level);
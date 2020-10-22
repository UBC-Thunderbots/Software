#pragma once
#include "shared/proto/robot_log_msg.nanopb.h"

/**
 * Initializes the logger with the given robot id and the log handler function
 *
 * @param robot_id The robot id the logs are coming from
 * @param robot_log_msg_handler The function to handle RobotLog protos
 */
void app_logger_init(unsigned robot_id,
                     void (*robot_log_msg_handler)(TbotsProto_RobotLog log_msg));

/**
 * Creates a new RobotLog proto and calls the the robot_log_msg_handler
 *
 * NOTE: If the log msg is longer than the max_size defined in the proto
 * the log will be truncated.
 *
 * @param file_name The file this log is located
 * @param line_number The line number where this log is located
 * @param log_level The severity level of this log
 * @param format The format string followed by printf style args
 */
void app_logger_log(const char *file_name, unsigned line_number,
                    TbotsProto_LogLevel log_level, const char *format, ...);

#define TLOG_DEBUG(...)                                                                  \
    app_logger_log(__FILE__, __LINE__, TbotsProto_LogLevel_DEBUG, __VA_ARGS__)
#define TLOG_INFO(...)                                                                   \
    app_logger_log(__FILE__, __LINE__, TbotsProto_LogLevel_INFO, __VA_ARGS__)
#define TLOG_WARN(...)                                                                   \
    app_logger_log(__FILE__, __LINE__, TbotsProto_LogLevel_WARN, __VA_ARGS__)
#define TLOG_FATAL(...)                                                                  \
    app_logger_log(__FILE__, __LINE__, TbotsProto_LogLevel_FATAL, __VA_ARGS__)

#pragma once
#include "proto/robot_log_msg.nanopb.h"

/**
 * Initializes the logger with the given robot id and the log handler function
 *
 * @param robot_id The robot id the logs are coming from
 * @param robot_log_msg_handler The function to handle RobotLog protos
 */
void app_logger_init(unsigned robot_id,
                     void (*robot_log_msg_handler)(TbotsProto_RobotLog log_msg));

/**
 * Creates a new RobotLog proto and calls the robot_log_msg_handler
 *
 * NOTE: If the log msg is longer than the max_size defined in the proto
 * the log will be truncated.
 *
 * @param file_name The file where the log will be printed
 * @param line_number The line where the log will be printed
 * @param log_level The severity level of this log
 * @param format The format string followed by printf style args
 */
void app_logger_log(const char *file_name, unsigned line_number,
                    TbotsProto_LogLevel log_level, const char *format, ...);


// `__LINE__` and `__FILE__` are predefined macros that the preprocessor will replace
// with the line number and file name of where the log is located.
#define TLOG_DEBUG(...)                                                                  \
    app_logger_log(__FILE__, __LINE__, TbotsProto_LogLevel_DEBUG, __VA_ARGS__)
#define TLOG_INFO(...)                                                                   \
    app_logger_log(__FILE__, __LINE__, TbotsProto_LogLevel_INFO, __VA_ARGS__)
#define TLOG_WARNING(...)                                                                \
    app_logger_log(__FILE__, __LINE__, TbotsProto_LogLevel_WARNING, __VA_ARGS__)
#define TLOG_FATAL(...)                                                                  \
    app_logger_log(__FILE__, __LINE__, TbotsProto_LogLevel_FATAL, __VA_ARGS__)

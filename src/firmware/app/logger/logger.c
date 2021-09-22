#include "firmware/app/logger/logger.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "proto/robot_log_msg.nanopb.h"

typedef struct Logger
{
    unsigned robot_id;
    void (*robot_log_msg_handler)(TbotsProto_RobotLog log_msg);
} Logger_t;

// NOTE: code-style-guide.md explicitly advises against using static variables in the
// app layer to prevent undefined behaviour when running the firmware in the simulator.
//
// However, the logger is the _only_ exception to that rule, as dependency injecting
// the logger into every object that wants to log, is simply overkill. Ontop of that,
// we cannot have useful log macros like TLOG_WARN(...), TLOG_ERROR(...), etc...
static Logger_t logger;

void app_logger_init(unsigned robot_id,
                     void (*robot_log_msg_handler)(TbotsProto_RobotLog log_msg))
{
    logger.robot_id              = robot_id;
    logger.robot_log_msg_handler = robot_log_msg_handler;
}

void app_logger_log(const char *file_name, unsigned line_number,
                    TbotsProto_LogLevel log_level, const char *format, ...)
{
    // TODO update timestamp: https://github.com/UBC-Thunderbots/Software/issues/1518
    TbotsProto_RobotLog robot_log = TbotsProto_RobotLog_init_zero;
    robot_log.robot_id            = logger.robot_id;
    robot_log.log_level           = log_level;
    robot_log.line_number         = line_number;
    strcpy(robot_log.file_name, file_name);

    // format the string with the variable args and load it into the RobotLog proto
    // see https://www.cprogramming.com/tutorial/c/lesson17.html for more info on VA
    va_list variable_argument_list;
    va_start(variable_argument_list, format);
    vsprintf(robot_log.log_msg, format, variable_argument_list);
    va_end(variable_argument_list);

    logger.robot_log_msg_handler(robot_log);
}

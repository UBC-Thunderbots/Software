#include "firmware_new/logger/logger.h"

#include <stdlib.h>

typedef struct Logger
{
    void (*handle_robot_log_msg)(TbotsProto_RobotLog);
} Logger_t;

Logger_t* app_logger_create(void (*handle_robot_log_msg)(TbotsProto_RobotLog))
{
    Logger_t* profile             = (Logger_t*)malloc(sizeof(Logger_t));
    profile->handle_robot_log_msg = handle_robot_log_msg;
    return profile;
}

void app_logger_log(const Logger_t* logger, TbotsProto_RobotLog log_msg)
{
    logger->handle_robot_log_msg(log_msg);
}

void app_logger_destroy(Logger_t* logger)
{
    free(logger);
}

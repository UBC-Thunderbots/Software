#include "firmware_new/logger/logger.h"

typedef struct Logger
{
    void (*handle_robot_log_msg)(TbotsProto_RobotLog);
} Logger_t;

Logger_t* app_logger_create(void (*handle_robot_log_msg)(TbotsProto_RobotLog));
void app_logger_log(TbotsProto_RobotLog log_msg);
void app_logger_destroy(Logger_t* logger);

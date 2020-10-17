#pragma once
#include "shared/proto/robot_log_msg.nanopb.h"

typedef struct Logger Logger_t;
Logger_t* app_logger_create(void (*handle_robot_log_msg)(TbotsProto_RobotLog));
void app_logger_log(const Logger_t* logger, TbotsProto_RobotLog log_msg);
void app_logger_destroy(Logger_t* logger);

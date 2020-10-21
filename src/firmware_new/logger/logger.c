#include "firmware_new/logger/logger.h"
#include "shared/proto/robot_log_msg.nanopb.h"

#include <stdlib.h>

static uint8_t robot_id_;
static void (*handle_robot_log_msg_)(TbotsProto_RobotLog);

void app_logger_init(uint8_t robot_id, void (*handle_robot_log_msg)(TbotsProto_RobotLog))
{
    robot_id_ = robot_id;
    handle_robot_log_msg_ = handle_robot_log_msg;
}

void app_logger_log(TbotsProto_RobotLog log_msg)
{
    handle_robot_log_msg_(log_msg);
}

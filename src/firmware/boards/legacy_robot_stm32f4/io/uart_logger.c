
#include "uart_logger.h"

#include <stdio.h>

#include "proto/robot_log_msg.nanopb.h"


void io_uart_logger_handleRobotLog(TbotsProto_RobotLog log)
{
    printf("[%s:%d] %s", log.file_name, log.line_number, log.log_msg);
}

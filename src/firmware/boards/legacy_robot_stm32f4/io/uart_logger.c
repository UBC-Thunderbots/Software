
#include "uart_logger.h"

#include <stdio.h>

#include "shared/proto/robot_log_msg.nanopb.h"


void io_uart_logger_handle_robot_log(TbotsProto_RobotLog log)
{
    printf("[%s:%d] %s", log.file_name, log.line_number, log.log_msg);
}

#pragma once

#include "shared/proto/robot_log_msg.nanopb.h"

/**
 * Redirects RobotLogs to a standard printf.
 *
 * @param log The log msg to print over UART
 */
void io_uart_logger_handle_robot_log(TbotsProto_RobotLog log);

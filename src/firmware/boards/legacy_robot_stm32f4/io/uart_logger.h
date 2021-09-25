#pragma once

#include "proto/robot_log_msg.nanopb.h"

/**
 * Redirects RobotLogs to a standard printf.
 *
 * @param log The log msg to print over UART
 */
void io_uart_logger_handleRobotLog(TbotsProto_RobotLog log);

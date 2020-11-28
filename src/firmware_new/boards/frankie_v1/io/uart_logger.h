#pragma once
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "firmware_new/boards/frankie_v1/usart.h"
#include "shared/proto/robot_log_msg.nanopb.h"

/**
 * Initialize the UART logger
 *
 * @param uart_handle The UART handle to send the logs over
 */
void io_uart_logger_init(UART_HandleTypeDef* uart_handle);

/**
 * Transfers the robot_log over UART.
 *
 * @param robot_log The log msg to send
 */
void io_uart_logger_handle_robot_log(TbotsProto_RobotLog robot_log);

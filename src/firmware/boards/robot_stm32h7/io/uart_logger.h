#pragma once
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "firmware/boards/robot_stm32h7/usart.h"
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
void io_uart_logger_handleRobotLog(TbotsProto_RobotLog robot_log);

/**
 * Converts the TbotsProto_LogLevel enum to a string
 *
 * @param log_level The log level to convert to a string
 * @returns converted string
 */
const char* io_uart_logger_convertLogLevelEnumToString(TbotsProto_LogLevel log_level);

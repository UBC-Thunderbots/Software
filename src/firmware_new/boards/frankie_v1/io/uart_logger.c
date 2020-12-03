#include "firmware_new/boards/frankie_v1/io/uart_logger.h"

#include <stdio.h>

#include "firmware_new/boards/frankie_v1/usart.h"
#include "shared/proto/robot_log_msg.nanopb.h"

static char g_robot_log_buffer[TbotsProto_RobotLog_size];
static UART_HandleTypeDef* g_uart_handle;

void io_uart_logger_init(UART_HandleTypeDef* uart_handle)
{
    g_uart_handle = uart_handle;
}

void io_uart_logger_handleRobotLog(TbotsProto_RobotLog robot_log)
{
    int size = sprintf(g_robot_log_buffer, "[%s][%s:%ld]: %s\r\n",
                       io_uart_logger_convertLogLevelEnumToString(log_level),
                       robot_log.file_name, robot_log.line_number, robot_log.log_msg);

    HAL_UART_Transmit(g_uart_handle, (uint8_t*)g_robot_log_buffer, (uint16_t)size,
                      HAL_MAX_DELAY);
}

const char* io_uart_logger_convertLogLevelEnumToString(TbotsProto_LogLevel log_level)
{
    switch(log_level)
    {
        case TbotsProto_LogLevel_DEBUG:
        {
            return "DEBUG";
        }
        case TbotsProto_LogLevel_INFO:
        {
            return "INFO";
        }
        case TbotsProto_LogLevel_WARNING:
        {
            return "WARNING";
        }
        case TbotsProto_LogLevel_FATAL:
        {
            return "FATAL";
        }
        default:
        {
            return "UNKOWN";
        }
    }
}


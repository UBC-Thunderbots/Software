#include "firmware/boards/frankie_stm32h7/io/uart_logger.h"

#include <stdio.h>

#include "firmware/boards/frankie_stm32h7/usart.h"
#include "shared/proto/robot_log_msg.nanopb.h"

static char g_robot_log_buffer[TbotsProto_RobotLog_size];
static UART_HandleTypeDef* g_uart_handle;

void io_uart_logger_init(UART_HandleTypeDef* uart_handle)
{
    g_uart_handle = uart_handle;

    // clear the console by sending the \x1b[2J ANSI clear command
    HAL_UART_Transmit(g_uart_handle, (uint8_t*)"\x1b[2J", 7, HAL_MAX_DELAY);
}

void io_uart_logger_handleRobotLog(TbotsProto_RobotLog robot_log)
{
    // the \033[0m is an ANSI escape sequence that tells the console to
    // clear any colors we have set for the log level
    int size = sprintf(g_robot_log_buffer, "%s[%s:%ld]: %s\033[0m\r\n",
                       io_uart_logger_convertLogLevelEnumToString(robot_log.log_level),
                       robot_log.file_name, robot_log.line_number, robot_log.log_msg);

    HAL_UART_Transmit(g_uart_handle, (uint8_t*)g_robot_log_buffer, (uint16_t)size,
                      HAL_MAX_DELAY);
}

const char* io_uart_logger_convertLogLevelEnumToString(TbotsProto_LogLevel log_level)
{
    switch (log_level)
    {
        case TbotsProto_LogLevel_DEBUG:
        {
            return "\033[37m[ DEBUG ]";
        }
        case TbotsProto_LogLevel_INFO:
        {
            return "\033[36m[ INFO  ]";
        }
        case TbotsProto_LogLevel_WARNING:
        {
            return "\033[33m[WARNING]";
        }
        case TbotsProto_LogLevel_FATAL:
        {
            return "\033[31m[ FATAL ]";
        }
        default:
        {
            return "[ UNKNOWN ]";
        }
    }
}

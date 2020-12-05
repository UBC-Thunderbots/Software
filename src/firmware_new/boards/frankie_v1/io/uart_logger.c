#include "firmware_new/boards/frankie_v1/io/uart_logger.h"

#include <stdio.h>

#include "firmware_new/boards/frankie_v1/usart.h"
#include "shared/proto/robot_log_msg.nanopb.h"

static char robot_log_buffer[TbotsProto_RobotLog_size];
static UART_HandleTypeDef* huart;

void io_uart_logger_init(UART_HandleTypeDef* uart_handle)
{
    huart = uart_handle;
}

void io_uart_logger_handle_robot_log(TbotsProto_RobotLog robot_log)
{
    const char* log_level;

    if (robot_log.log_level == TbotsProto_LogLevel_DEBUG)
    {
        log_level = "\033[36m[ DEBUG ]";
    }
    else if (robot_log.log_level == TbotsProto_LogLevel_INFO)
    {
        log_level = "\033[37m[ INFO  ]";
    }
    else if (robot_log.log_level == TbotsProto_LogLevel_WARNING)
    {
        log_level = "\033[33m[ WARN  ]";
    }
    else if (robot_log.log_level == TbotsProto_LogLevel_FATAL)
    {
        log_level = "\033[31m[ FATAL ]";
    }
    else
    {
        log_level = "UNKNOWN";
    }

    int size = sprintf(robot_log_buffer, "%s[%s:%ld]: %s\033[0m\r\n", log_level, robot_log.file_name,
                       robot_log.line_number, robot_log.log_msg);

    HAL_UART_Transmit(huart, (uint8_t*)robot_log_buffer, (uint16_t)size, HAL_MAX_DELAY);
}

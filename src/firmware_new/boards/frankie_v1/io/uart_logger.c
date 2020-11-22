#include "firmware_new/boards/frankie_v1/io/uart_logger.h"

#include <stdio.h>

#include "firmware_new/boards/frankie_v1/usart.h"
#include "shared/proto/robot_log_msg.nanopb.h"

static char buffer[200];
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
        log_level = "DEBUG";
    }
    else if (robot_log.log_level == TbotsProto_LogLevel_INFO)
    {
        log_level = "INFO";
    }
    else if (robot_log.log_level == TbotsProto_LogLevel_WARNING)
    {
        log_level = "WARNING";
    }
    else if (robot_log.log_level == TbotsProto_LogLevel_FATAL)
    {
        log_level = "FATAL";
    }
    else
    {
        log_level = "UNKOWN";
    }

    int size = sprintf(buffer, "[%s][%s:%ld]: %s\r\n", log_level, robot_log.file_name,
                       robot_log.line_number, robot_log.log_msg);

    HAL_UART_Transmit(huart, (uint8_t*)buffer, (uint16_t)size, HAL_MAX_DELAY);
}

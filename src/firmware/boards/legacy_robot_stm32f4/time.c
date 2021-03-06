#include "firmware/boards/legacy_robot_stm32f4/time.h"

#include <FreeRTOS.h>


float get_current_freertos_tick_time_seconds()
{
    return ((float)xTaskGetTickCount()) * portTICK_PERIOD_MS;
}

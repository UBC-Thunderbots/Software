#pragma once

/**
 * Get the current time in seconds based on the current freertos tick
 *
 * NOTE: this is only as granular as the tick rate of the FreeRTOS task that
 *       it is called from
 *
 * @return The current time in seconds
 */
float get_current_freertos_tick_time_seconds();

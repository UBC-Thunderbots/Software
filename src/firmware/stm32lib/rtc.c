/**
 * \defgroup RTC Real Time Clock Functions
 *
 * \brief These functions track real-world time, once initialized with a single
 * timestamp.
 *
 * The RTC uses timestamps measured in milliseconds.
 *
 * \{
 */

#if STM32LIB_USE_FREERTOS
#include "rtc.h"

#include <FreeRTOS.h>
#include <registers/systick.h>
#include <stdbool.h>
#include <task.h>

#include "init.h"

/**
 * \brief The number of microseconds per FreeRTOS tick.
 */
#define US_PER_TICK (portTICK_PERIOD_MS * 1000)

/**
 * \brief Whether the RTC has ever been set.
 */
static bool rtc_valid = false;

/**
 * \brief The RTC value corresponding to a FreeRTOS tick count of zero.
 */
static uint64_t rtc_base;

/**
 * \brief Gets the current real time.
 *
 * \return the current timestamp, or zero if not set yet
 */
uint64_t rtc_get(void)
{
    bool valid;
    uint64_t base;
    uint32_t systick;
    TickType_t ticks1, ticks2;

    taskENTER_CRITICAL();
    valid = rtc_valid;
    base  = rtc_base;
    taskEXIT_CRITICAL();

    if (valid)
    {
        // Iterate until we capture the upper and lower parts without a
        // rollover.
        ticks2 = xTaskGetTickCount();
        do
        {
            ticks1  = ticks2;
            systick = SYSTICK.CVR;
            ticks2  = xTaskGetTickCount();
        } while (ticks1 != ticks2);

        // Compose the whole value.
        return (uint64_t)ticks1 * US_PER_TICK + systick / init_specs()->cpu_frequency +
               base;
    }
    else
    {
        return 0;
    }
}

/**
 * \brief Sets the current real time.
 *
 * \param[in] stamp the timestamp
 */
void rtc_set(uint64_t stamp)
{
    TickType_t ticks = xTaskGetTickCount();
    taskENTER_CRITICAL();
    rtc_base  = stamp - (uint64_t)ticks * US_PER_TICK;
    rtc_valid = true;
    taskEXIT_CRITICAL();
}
#endif

/**
 * \}
 */

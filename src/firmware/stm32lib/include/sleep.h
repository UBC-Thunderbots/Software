/**
 * \defgroup SLEEP Busy-wait functions
 *
 * These functions are only for use in non-FreeRTOS-based code.
 * They assume that the system tick timer is set to overflow every microsecond.
 *
 * @{
 */

#ifndef STM32LIB_SLEEP_H
#define STM32LIB_SLEEP_H

/**
 * \cond INTERNAL
 */
void sleep_systick_overflows(unsigned long ticks);
/**
 * \endcond
 */

/**
 * \brief Sleeps for some number of milliseconds.
 *
 * \param x the number of milliseconds to sleep
 */
#define sleep_ms(x) sleep_us((x)*1000UL)

/**
 * \brief Sleeps for some number of microseconds.
 *
 * \param x the number of microseconds to sleep
 */
#define sleep_us(x) sleep_systick_overflows((x))

#endif

/**
 * @}
 */

#include "led.h"

#include <FreeRTOS.h>
#include <registers/gpio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <timers.h>
#include <unused.h>

#include "pins.h"

/**
 * \brief The width of a blink half-period.
 *
 * This is the number of milliseconds to leave an LED in the non-rest state
 * when it blinks, and to leave the LED in the rest state after it returns from
 * a blink before starting the next blink.
 */
#define LED_BLINK_TIME_MS 25

/**
 * \brief The length of the lamp test, in milliseconds.
 */
#define LED_LAMP_TEST_TIME_MS 500

/**
 * \brief A bitmask of whether each LED is on or off when at rest.
 */
static unsigned int led_rest_state = 0;

/**
 * \brief A bitmask of whether each LED has been asked to blink since its last
 * blink started.
 */
static unsigned int led_blink_pending = 0;

/**
 * \brief The toggle, which indicates whether a blink is currently active.
 */
static bool led_blink_toggle = false;

/**
 * \brief The number of ticks left before the startup lamp test is complete.
 */
static unsigned int led_lamp_test_counter = LED_LAMP_TEST_TIME_MS / LED_BLINK_TIME_MS;
_Static_assert(!(LED_LAMP_TEST_TIME_MS % LED_BLINK_TIME_MS),
               "Lamp test time must be a multiple of blink time");

/**
 * \brief The GPIOs to which the LEDs are attached.
 */
static const struct
{
    volatile GPIO_BSRR_t *bsrr;
    uint32_t mask;
} LED_PINS[] = {
    [LED_POWER] = {&GPIOB.BSRR, 1 << 12},
    [LED_TX]    = {&GPIOB.BSRR, 1 << 13},
    [LED_RX]    = {&GPIOB.BSRR, 1 << 14},
};

/**
 * \brief The timer callback that updates the LEDs.
 */
static void led_tick(TimerHandle_t UNUSED(timer))
{
    if (led_lamp_test_counter)
    {
        // Leave all LEDs on until lamp test complete.
        --led_lamp_test_counter;
    }
    else
    {
        led_blink_toggle   = !led_blink_toggle;
        unsigned int state = led_rest_state;
        if (led_blink_toggle)
        {
            state ^= led_blink_pending;
            led_blink_pending = 0;
        }
        for (size_t i = 0; i < sizeof(LED_PINS) / sizeof(*LED_PINS); ++i)
        {
            if (state & (1 << i))
            {
                GPIO_BSRR_t bsrr  = {.BS = LED_PINS[i].mask, .BR = 0};
                *LED_PINS[i].bsrr = bsrr;
            }
            else
            {
                GPIO_BSRR_t bsrr  = {.BS = 0, .BR = LED_PINS[i].mask};
                *LED_PINS[i].bsrr = bsrr;
            }
        }
    }
}

/**
 * \brief Initializes the LED subsystem.
 */
void led_init(void)
{
    static StaticTimer_t timer_storage;
    TimerHandle_t timer =
        xTimerCreateStatic("led", LED_BLINK_TIME_MS / portTICK_PERIOD_MS, pdTRUE, 0,
                           &led_tick, &timer_storage);
    if (!xTimerStart(timer, portMAX_DELAY))
    {
        abort();
    }
}

/**
 * \brief Turns on an LED.
 *
 * \param[in] led the LED to turn on
 */
void led_on(led_t led)
{
    __atomic_or_fetch(&led_rest_state, 1 << led, __ATOMIC_RELAXED);
    __atomic_signal_fence(__ATOMIC_SEQ_CST);
    if (!__atomic_load_n(&led_lamp_test_counter, __ATOMIC_RELAXED))
    {
        GPIO_BSRR_t bsrr    = {.BS = LED_PINS[led].mask, .BR = 0};
        *LED_PINS[led].bsrr = bsrr;
    }
}

/**
 * \brief Turns off an LED.
 *
 * \param[in] led the LED to turn off
 */
void led_off(led_t led)
{
    __atomic_and_fetch(&led_rest_state, ~(1 << led), __ATOMIC_RELAXED);
    __atomic_signal_fence(__ATOMIC_SEQ_CST);
    if (!__atomic_load_n(&led_lamp_test_counter, __ATOMIC_RELAXED))
    {
        GPIO_BSRR_t bsrr    = {.BS = 0, .BR = LED_PINS[led].mask};
        *LED_PINS[led].bsrr = bsrr;
    }
}

/**
 * \brief Blinks an LED.
 *
 * \param[in] led the LED to blink
 */
void led_blink(led_t led)
{
    __atomic_or_fetch(&led_blink_pending, 1 << led, __ATOMIC_RELAXED);
}

/**
 * \brief Immediately ends the startup lamp test.
 *
 * This may be useful in an exception handler that needs to blink the LEDs to
 * indicate a crash, or if the device needs to enter a low power state shortly
 * after startup.
 *
 * This function can safely be called after the lamp test has ended.
 */
void led_end_lamp_test_early(void)
{
    __atomic_store_n(&led_lamp_test_counter, 0, __ATOMIC_RELAXED);
}

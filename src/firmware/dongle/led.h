#ifndef LED_H
#define LED_H

/**
 * \file
 *
 * \brief Controls the LEDs.
 */

/**
 * \brief The LEDs installed on the device.
 */
typedef enum
{
    LED_POWER,
    LED_TX,
    LED_RX,
} led_t;

void led_init(void);
void led_on(led_t led);
void led_off(led_t led);
void led_blink(led_t led);
void led_end_lamp_test_early(void);

#endif

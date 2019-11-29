#ifndef LEDS_H
#define LEDS_H

#include "pins.h"

/**
 * \ingroup LEDS
 *
 * \brief The possible modes for the test LEDs.
 */
typedef enum
{
    LEDS_TEST_MODE_NORMAL,    ///< LED 0 shows break beam status, LED 1 shows auto-chick
                              ///< armed, LED 2 off
    LEDS_TEST_MODE_CONSTANT,  ///< LEDs show a fixed pattern
    LEDS_TEST_MODE_HALL,      ///< LEDs show the states of a Hall sensor
    LEDS_TEST_MODE_ENCODER,   ///< LEDs 0 and 1 show the states of an optical encoder, LED
                              ///< 2 off
} leds_test_mode_t;

/**
 * \ingroup LEDs
 *
 * \brief Sets the state of the status LED.
 *
 * \param[in] level the level to set
 */
#define leds_status_set(level) gpio_set_output(PIN_LED_STATUS, level)

/**
 * \ingroup LEDs
 *
 * \brief Sets the state of the link LED.
 *
 * \param[in] level the level to set
 */
#define leds_link_set(level) gpio_set_output(PIN_LED_LINK, level)

void leds_init(void);
void leds_tick(void);
void leds_test_set_mode(leds_test_mode_t mode, unsigned int param);

#endif

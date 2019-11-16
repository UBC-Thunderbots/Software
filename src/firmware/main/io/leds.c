/**
 * \defgroup LEDS LED Control Functions
 *
 * \brief These functions control the LEDs.
 *
 * @{
 */

#include "leds.h"

#include "adc.h"
#include "breakbeam.h"
#include "chicker.h"
#include "icb.h"

static const struct
{
    volatile GPIO_t *gpio;
    unsigned int bit;
} ENCODER_PINS[4U][2U] = {
    {{&PIN_ENCODER0A}, {&PIN_ENCODER0B}},
    {{&PIN_ENCODER1A}, {&PIN_ENCODER1B}},
    {{&PIN_ENCODER2A}, {&PIN_ENCODER2B}},
    {{&PIN_ENCODER3A}, {&PIN_ENCODER3B}},
};
static leds_test_mode_t test_mode = LEDS_TEST_MODE_NORMAL;
static unsigned int test_param;

/**
 * \brief Initializes the LEDs.
 */
void leds_init(void)
{
    // SAFETY CRITICAL: Charged LED must show proper value very early on!
    gpio_set_output(PIN_LED_CHARGED, adc_capacitor() > CHICKER_CHARGED_THRESHOLD);

    // Other LEDs should be off now.
    leds_status_set(false);
    leds_link_set(false);
}

/**
 * \brief Updates the states of the LEDs.
 */
void leds_tick(void)
{
    // Update the charged LED.
    gpio_set_output(PIN_LED_CHARGED, adc_capacitor() > CHICKER_CHARGED_THRESHOLD);

    // Update the test LEDs.
    leds_test_mode_t mode = __atomic_load_n(&test_mode, __ATOMIC_RELAXED);
    unsigned int param    = __atomic_load_n(&test_param, __ATOMIC_RELAXED);
    static uint8_t packet[2U];
    switch (mode)
    {
        case LEDS_TEST_MODE_NORMAL:
            packet[0U] = 0U;
            packet[1U] = (chicker_auto_armed() ? 0x02U : 0x00U) |
                         (breakbeam_interrupted() ? 0x01U : 0x00U);
            break;

        case LEDS_TEST_MODE_CONSTANT:
            packet[0U] = 0U;
            packet[1U] = param;
            break;

        case LEDS_TEST_MODE_HALL:
            packet[0U] = 1U;
            packet[1U] = param;
            break;

        case LEDS_TEST_MODE_ENCODER:
            packet[0U] = 0U;
            packet[1U] = 0U;
            for (unsigned int bit = 0U; bit != 2U; ++bit)
            {
                if (gpio_get_input_raw(*ENCODER_PINS[param & 3U][bit].gpio,
                                       ENCODER_PINS[param & 3U][bit].bit))
                {
                    packet[1U] |= 1U << bit;
                }
            }
            break;
    }
    icb_send(ICB_COMMAND_WRITE_LEDS, packet, sizeof(packet));
}

/**
 * \brief Selects what to display on the test LEDs.
 *
 * \param[in] mode the class of data to display
 * \param[in] param a mode-specific parameter, such as a constant pattern for \ref
 * LEDS_TEST_MODE_CONSTANT or a motor index for \ref LEDS_TEST_MODE_HALL or \ref
 * LEDS_TEST_MODE_ENCODER
 */
void leds_test_set_mode(leds_test_mode_t mode, unsigned int param)
{
    __atomic_store_n(&test_mode, mode, __ATOMIC_RELAXED);
    __atomic_store_n(&test_param, param, __ATOMIC_RELAXED);
}

/**
 * @}
 */

/**
 * \addtogroup GPIO
 * @{
 */

#ifndef STM32LIB_GPIO_H
#define STM32LIB_GPIO_H

#include <registers/gpio.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * \brief Gets the mode of an I/O pin.
 *
 * \param[in] pin the pin
 *
 * \return the mode
 */
#define gpio_get_mode(pin) gpio_get_mode_raw(pin)

/**
 * \brief Sets the mode of an I/O pin.
 *
 * \param[in] pin the pin
 *
 * \param[in] mode the mode
 */
#define gpio_set_mode(pin, mode) gpio_set_mode_raw(pin, mode)

/**
 * \brief Checks if an I/O pin is push-pull.
 *
 * \param[in] pin the pin
 *
 * \return \c true if the port is push-pull, or \c false if open-drain
 */
#define gpio_is_pp(pin) gpio_is_pp_raw(pin)

/**
 * \brief Checks if an I/O pin is open-drain.
 *
 * \param[in] pin the pin
 *
 * \return \c true if the port is open-drain, or \c false if push-pull
 */
#define gpio_is_od(pin) gpio_is_od_raw(pin)

/**
 * \brief Sets an I/O pin to push-pull mode.
 *
 * \param[in] pin the pin
 */
#define gpio_set_pp(pin) gpio_set_pp_raw(pin)

/**
 * \brief Sets an I/O pin to open-drain mode.
 *
 * \param[in] pin the pin
 */
#define gpio_set_od(pin) gpio_set_od_raw(pin)

/**
 * \brief Gets the drive speed of an I/O pin.
 *
 * \param[in] pin the pin
 *
 * \return the speed
 */
#define gpio_get_speed(pin) gpio_get_speed_raw(pin)

/**
 * \brief Sets the drive speed of an I/O pin.
 *
 * \param[in] pin the pin
 *
 * \param[in] speed the speed
 */
#define gpio_set_speed(pin, speed) gpio_set_speed_raw(pin, speed)

/**
 * \brief Gets the pull direction of an I/O pin.
 *
 * \param[in] pin the pin
 *
 * \return the pull direction
 */
#define gpio_get_pupd(pin) gpio_get_pupd_raw(pin)

/**
 * \brief Sets the pull direction of an I/O pin.
 *
 * \param[in] pin the pin
 *
 * \param[in] pupd the pull direction
 */
#define gpio_set_pupd(pin, pupd) gpio_set_pupd_raw(pin, pupd)

/**
 * \brief Gets the alternate function attached to an I/O pin.
 *
 * \param[in] pin the pin
 *
 * \return the alternate function
 */
#define gpio_get_af(pin) gpio_get_af_raw(pin)

/**
 * \brief Selects an alternate function on an I/O pin.
 *
 * \param[in] pin the pin
 *
 * \param[in] af the alternate function
 */
#define gpio_set_af(pin, af) gpio_set_af_raw(pin, af)

/**
 * \brief Gets the state of an input pin.
 *
 * \param[in] pin the pin
 *
 * \return the state, \c true or \c false
 */
#define gpio_get_input(pin) gpio_get_input_raw(pin)

/**
 * \brief Gets the state of an output driver.
 *
 * \param[in] pin the pin
 *
 * \return the state, \c true or \c false
 */
#define gpio_get_output(pin) gpio_get_output_raw(pin)

/**
 * \brief Inverts an output driver.
 *
 * \param[in] pin the pin
 */
#define gpio_toggle(pin) gpio_toggle_raw(pin)

/**
 * \brief Resets a single I/O pin.
 *
 * \param[in] pin the pin
 */
#define gpio_reset(pin) gpio_reset_raw(pin)

/**
 * \brief Sets a single I/O pin.
 *
 * \param[in] pin the pin
 */
#define gpio_set(pin) gpio_set_raw(pin)

/**
 * \brief Sets or resets a single I/O pin.
 *
 * \param[in] pin the pin
 *
 * \param[in] level \c true to set the pin, or \c false to reset it
 */
#define gpio_set_output(pin, level) gpio_set_output_raw(pin, level)

/**
 * \brief Sets and resets a subset of the bits of a port.
 *
 * If a single bit is asked to both set and reset, it is set.
 *
 * \param[in] port the port
 *
 * \param[in] set the bitmask of bits to set
 *
 * \param[in] reset the bitmask of bits to clear
 */
#define gpio_set_reset_mask(port, set, reset)                                            \
    do                                                                                   \
    {                                                                                    \
        GPIO_BSRR_t tmp = {.BS = (set), .BR = (reset)};                                  \
        (port).BSRR     = tmp;                                                           \
    } while (0)

/**
 * \brief Gets the mode of an I/O pin.
 *
 * \param[in] port the port
 *
 * \param[in] bit the bit
 *
 * \return the mode
 */
#define gpio_get_mode_raw(port, bit)                                                     \
    ((GPIO_MODE_t)(((port).MODER >> (2 * (bit))) & GPIO_MODE_MASK))

/**
 * \brief Sets the mode of an I/O pin.
 *
 * \param[in] port the port
 *
 * \param[in] bit the bit
 *
 * \param[in] mode the mode
 */
#define gpio_set_mode_raw(port, bit, mode)                                               \
    do                                                                                   \
    {                                                                                    \
        (port).MODER =                                                                   \
            ((port).MODER & ~(GPIO_MODE_MASK << (2 * (bit)))) | ((mode) << (2 * (bit))); \
    } while (0)

/**
 * \brief Checks if an I/O pin is push-pull.
 *
 * \param[in] port the port
 *
 * \param[in] bit the bit
 *
 * \return \c true if the port is push-pull, or \c false if open-drain
 */
#define gpio_is_pp_raw(port, bit) ((((port).OTYPER >> (bit)) & 1) == GPIO_OTYPE_PP)

/**
 * \brief Checks if an I/O pin is open-drain.
 *
 * \param[in] port the port
 *
 * \param[in] bit the bit
 *
 * \return \c true if the port is open-drain, or \c false if push-pull
 */
#define gpio_is_od_raw(port, bit) ((((port).OTYPER >> (bit)) & 1) == GPIO_OTYPE_OD)

/**
 * \brief Sets an I/O pin to push-pull mode.
 *
 * \param[in] port the port
 *
 * \param[in] bit the bit
 */
#define gpio_set_pp_raw(port, bit)                                                       \
    do                                                                                   \
    {                                                                                    \
        (port).OTYPER =                                                                  \
            ((port).OTYPER & ~(GPIO_OTYPE_MASK << (bit))) | (GPIO_OTYPE_PP << (bit));    \
    } while (0)

/**
 * \brief Sets an I/O pin to open-drain mode.
 *
 * \param[in] port the port
 *
 * \param[in] bit the bit
 */
#define gpio_set_od_raw(port, bit)                                                       \
    do                                                                                   \
    {                                                                                    \
        (port).OTYPER =                                                                  \
            ((port).OTYPER & ~(GPIO_OTYPE_MASK << (bit))) | (GPIO_OTYPE_OD << (bit));    \
    } while (0)

/**
 * \brief Gets the drive speed of an I/O pin.
 *
 * \param[in] port the port
 *
 * \param[in] bit the bit
 *
 * \return the speed
 */
#define gpio_get_speed_raw(port, bit)                                                    \
    ((GPIO_OSPEED_t)(((port).OSPEEDR >> (2 * (bit))) & GPIO_OSPEED_MASK))

/**
 * \brief Sets the drive speed of an I/O pin.
 *
 * \param[in] port the port
 *
 * \param[in] bit the bit
 *
 * \param[in] speed the speed
 */
#define gpio_set_speed_raw(port, bit, speed)                                             \
    do                                                                                   \
    {                                                                                    \
        (port).OSPEEDR = ((port).OSPEEDR & ~(GPIO_SPEED_MASK << (2 * (bit)))) |          \
                         ((speed) << (2 * (bit)));                                       \
    } while (0)

/**
 * \brief Gets the pull direction of an I/O pin.
 *
 * \param[in] port the port
 *
 * \param[in] bit the bit
 *
 * \return the pull direction
 */
#define gpio_get_pupd_raw(port, bit)                                                     \
    ((GPIO_PUPD_t)(((port).PUPDR >> (2 * (bit))) & GPIO_PUPD_MASK));

/**
 * \brief Sets the pull direction of an I/O pin.
 *
 * \param[in] port the port
 *
 * \param[in] bit the bit
 *
 * \param[in] pupd the pull direction
 */
#define gpio_set_pupd_raw(port, bit, pupd)                                               \
    do                                                                                   \
    {                                                                                    \
        (port).PUPDR =                                                                   \
            ((port).PUPDR & ~(GPIO_PUPD_MASK << (2 * (bit)))) | ((pupd) << (2 * (bit))); \
    } while (0)

/**
 * \brief Gets the alternate function attached to an I/O pin.
 *
 * \param[in] port the port
 *
 * \param[in] bit the bit
 *
 * \return the alternate function
 */
#define gpio_get_af_raw(port, bit)                                                       \
    ((bit) <= 7 ? (((port).AFRL >> (4 * (bit))) & 15)                                    \
                : (((port).AFRH >> (4 * ((bit)-8))) & 15))

/**
 * \brief Selects an alternate function on an I/O pin.
 *
 * \param[in] port the port
 *
 * \param[in] bit the bit
 *
 * \param[in] af the alternate function
 */
#define gpio_set_af_raw(port, bit, af)                                                   \
    do                                                                                   \
    {                                                                                    \
        unsigned int shift_dist = (bit) <= 7 ? 4 * (bit) : 4 * ((bit)-8);                \
        if ((bit) <= 7)                                                                  \
        {                                                                                \
            (port).AFRL =                                                                \
                ((port).AFRL & ~(((uint32_t)15) << shift_dist)) | ((af) << shift_dist);  \
        }                                                                                \
        else                                                                             \
        {                                                                                \
            (port).AFRH =                                                                \
                ((port).AFRH & ~(((uint32_t)15) << shift_dist)) | ((af) << shift_dist);  \
        }                                                                                \
    } while (0)

/**
 * \brief Gets the state of an input pin.
 *
 * \param[in] port the port
 *
 * \param[in] bit the bit
 *
 * \return the state, \c true or \c false
 */
#define gpio_get_input_raw(port, bit) (!!((port).IDR & (1 << (bit))))

/**
 * \brief Gets the state of an output driver.
 *
 * \param[in] port the port
 *
 * \param[in] bit the bit
 *
 * \return the state, \c true or \c false
 */
#define gpio_get_output_raw(port, bit) (!!((port).ODR & (1 << (bit))))

/**
 * \brief Inverts an output driver.
 *
 * \param[in] port the port
 *
 * \param[in] bit the bit
 */
#define gpio_toggle_raw(port, bit)                                                       \
    do                                                                                   \
    {                                                                                    \
        (port).ODR ^= 1 << (bit);                                                        \
    } while (0)

/**
 * \brief Resets a single I/O pin.
 *
 * \param[in] port the port
 *
 * \param[in] bit the bit
 */
#define gpio_reset_raw(port, bit) gpio_set_reset_mask(port, 0, 1 << (bit))

/**
 * \brief Sets a single I/O pin.
 *
 * \param[in] port the port
 *
 * \param[in] bit the bit
 */
#define gpio_set_raw(port, bit) gpio_set_reset_mask(port, 1 << (bit), 0)

/**
 * \brief Sets or resets a single I/O pin.
 *
 * \param[in] port the port
 *
 * \param[in] bit the bit
 *
 * \param[in] level \c true to set the pin, or \c false to reset it
 */
#define gpio_set_output_raw(port, bit, level)                                            \
    gpio_set_reset_mask(port, (level) ? (1 << (bit)) : 0, 1 << (bit))

/**
 * \brief Describes the initial state of a single I/O pin.
 */
typedef struct __attribute__((packed))
{
    GPIO_MODE_t mode : 2;      ///< The mode
    GPIO_OTYPE_t otype : 1;    ///< The type of output driver
    GPIO_OSPEED_t ospeed : 2;  ///< The speed of the output driver
    GPIO_PUPD_t pupd : 2;      ///< The pull-up/pull-down configuration
    unsigned od : 1;           ///< The initial level in the output driver
    unsigned af : 4;           ///< The alternate function number
    bool unlock : 1;  ///< Whether to leave the pin configuration unlocked when locking
                      ///< the ports
} gpio_init_pin_t;

void gpio_init(const gpio_init_pin_t (*specs)[16U], size_t length);

#endif

/**
 * @}
 */

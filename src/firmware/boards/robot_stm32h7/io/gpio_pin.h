#pragma once

#include "firmware/boards/robot_stm32h7/Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal.h"

typedef struct GpioPin GpioPin_t;

/**
 * A GPIO pin being "Active High" or "Active Low" simply determines whether high or low
 * voltage corresponds to 1. So for example, "Active High" means that a high voltage
 * corresponds to 1, a low voltage corresponds to 0.
 */
typedef enum
{
    ACTIVE_HIGH,
    ACTIVE_LOW,
    INPUT,
} GpioPinActiveState;

/**
 * Create a GPIO pin
 *
 * Please note, the GPIO handler/pin must be configured before being passed to this
 * function. Setting it up in CubeMX should be sufficient.
 *
 * @param gpio_handler A GPIO handler which this pin is contained in. This must be kept
 *                     alive for the duration of the created GPIO pin object.
 * @param gpio_pin_index The index within the GPIO handler for this pin
 * @param active_state The active state of this pin
 *
 * @return An initialized GPIO pin, initially set to be inactive (0). Ownership of the
 *         returned pointer is taken by the caller.
 */
GpioPin_t* io_gpio_pin_create(GPIO_TypeDef* gpio_handler, uint16_t gpio_pin_index,
                              GpioPinActiveState active_state);

/**
 * Destroys the given GPIO pin
 * @param gpio_pin The GPIO pin to destroy.
 */
void io_gpio_pin_destroy(GpioPin_t* gpio_pin);

/**
 * Set the given GPIO pin to be active (1)
 * @param gpio_pin
 */
void io_gpio_pin_setActive(GpioPin_t* gpio_pin);

/**
 * Set the given GPIO pin to be inactive (0)
 * @param gpio_pin
 */
void io_gpio_pin_setInactive(GpioPin_t* gpio_pin);

uint32_t io_gpio_pin_getState(GpioPin_t* gpio_pin);

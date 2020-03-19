#pragma once

#include "firmware_new/boards/frankie_v1/Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal.h"

typedef struct GpioPin GpioPin_t;

// TODO: jdoc here, assume people do not know what "active high" and "active low"
//       mean (see, software ppl)
// TODO: better name for this, don't want ppl to get confused that this is the initial
//       state for the pin
// TODO: should prefix enum values with something to avoid conflicts
typedef enum
{
    ACTIVE_HIGH,
    ACTIVE_LOW
} GpioPinActiveState;

// TODO: jdoc here
// TODO: better names here
// TODO: remember to state requirements for stuff that needs to be configured before
//       being passed to this function
GpioPin_t* io_gpio_pin_create(GPIO_TypeDef* gpio_handler, uint16_t gpio_pin_index,
                              GpioPinActiveState active_state);

// TODO: jdoc here
void io_gpio_pin_destroy(GpioPin_t* gpio_pin);

// TODO: jdoc here
void io_gpio_pin_setActive(GpioPin_t* gpio_pin);

// TODO: jdoc here
void io_gpio_pin_setInactive(GpioPin_t* gpio_pin);

#include "firmware_new/boards/frankie_v1/io/gpio_pin.h"

#include <stdlib.h>

#include "firmware_new/boards/frankie_v1/stm32h7xx_hal_conf.h"

typedef struct GpioPin
{
    GPIO_TypeDef* gpio_handler;
    uint16_t gpio_pin_index;
    GpioPinActiveState active_state;
} GpioPin_t;

// TODO: jdoc here
// TODO: does this function _really_ save enough code to be worth it?
void io_gpio_setHALPinState(GpioPin_t* gpio_pin, GPIO_PinState pin_state)
{
    HAL_GPIO_WritePin(gpio_pin->gpio_handler, gpio_pin->gpio_pin_index, pin_state);
}

GpioPin_t* io_gpio_pin_create(GPIO_TypeDef* gpio_handler, uint16_t gpio_pin_index,
                              GpioPinActiveState active_state)
{
    GpioPin_t* gpio_pin = (GpioPin_t*)malloc(sizeof(GpioPin_t));

    gpio_pin->gpio_handler   = gpio_handler;
    gpio_pin->gpio_pin_index = gpio_pin_index;
    gpio_pin->active_state   = active_state;

    return gpio_pin;
}

// TODO: jdoc here
void io_gpio_pin_destroy(GpioPin_t* gpio_pin)
{
    free(gpio_pin);
}

// TODO: DRY with `setActive` and `setInactive`

// TODO: jdoc here
void io_gpio_pin_setActive(GpioPin_t* gpio_pin)
{
    switch (gpio_pin->active_state)
    {
        case ACTIVE_HIGH:
            io_gpio_setHALPinState(gpio_pin, GPIO_PIN_SET);
            return;
        case ACTIVE_LOW:
            io_gpio_setHALPinState(gpio_pin, GPIO_PIN_RESET);
            return;
    }
}

// TODO: jdoc here
void io_gpio_pin_setInactive(GpioPin_t* gpio_pin)
{
    switch (gpio_pin->active_state)
    {
        case ACTIVE_HIGH:
            io_gpio_setHALPinState(gpio_pin, GPIO_PIN_RESET);
            return;
        case ACTIVE_LOW:
            io_gpio_setHALPinState(gpio_pin, GPIO_PIN_SET);
            return;
    }
}

#include "firmware/boards/robot_stm32h7/io/gpio_pin.h"

#include <stdlib.h>

#include "firmware/boards/robot_stm32h7/stm32h7xx_hal_conf.h"

typedef struct GpioPin
{
    GPIO_TypeDef* gpio_handler;
    uint16_t gpio_pin_index;
    GpioPinActiveState active_state;
} GpioPin_t;

/**
 * Set raw HAL state for the given GPIO pin
 * @param gpio_pin The pin to set the state for
 * @param pin_state The state to set the pin to
 */
void io_gpio_pin_setHALPinState(GpioPin_t* gpio_pin, GPIO_PinState pin_state);

GpioPin_t* io_gpio_pin_create(GPIO_TypeDef* gpio_handler, uint16_t gpio_pin_index,
                              GpioPinActiveState active_state)
{
    GpioPin_t* gpio_pin = (GpioPin_t*)malloc(sizeof(GpioPin_t));

    gpio_pin->gpio_handler   = gpio_handler;
    gpio_pin->gpio_pin_index = gpio_pin_index;
    gpio_pin->active_state   = active_state;

    io_gpio_pin_setInactive(gpio_pin);

    return gpio_pin;
}

void io_gpio_pin_destroy(GpioPin_t* gpio_pin)
{
    free(gpio_pin);
}

void io_gpio_pin_setActive(GpioPin_t* gpio_pin)
{
    switch (gpio_pin->active_state)
    {
        case ACTIVE_HIGH:
            io_gpio_pin_setHALPinState(gpio_pin, GPIO_PIN_SET);
            return;
        case ACTIVE_LOW:
            io_gpio_pin_setHALPinState(gpio_pin, GPIO_PIN_RESET);
            return;
        case INPUT:
            return;
    }
}

void io_gpio_pin_setInactive(GpioPin_t* gpio_pin)
{
    switch (gpio_pin->active_state)
    {
        case ACTIVE_HIGH:
            io_gpio_pin_setHALPinState(gpio_pin, GPIO_PIN_RESET);
            return;
        case ACTIVE_LOW:
            io_gpio_pin_setHALPinState(gpio_pin, GPIO_PIN_SET);
            return;
        case INPUT:
            return;
    }
}

void io_gpio_pin_setHALPinState(GpioPin_t* gpio_pin, GPIO_PinState pin_state)
{
    HAL_GPIO_WritePin(gpio_pin->gpio_handler, gpio_pin->gpio_pin_index, pin_state);
}

uint32_t io_gpio_pin_getState(GpioPin_t* gpio_pin)
{
    return HAL_GPIO_ReadPin(gpio_pin->gpio_handler, gpio_pin->gpio_pin_index);
}

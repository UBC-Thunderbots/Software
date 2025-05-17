#pragma once

#include <memory>

#include "software/embedded/constants/constants.h"
#include "software/embedded/gpio/gpio.h"
#include "software/embedded/gpio/gpio_char_dev.h"
#include "software/embedded/gpio/gpio_sysfs.h"

/**
 * Helper function to setup a GPIO pin. Selects the appropriate GPIO implementation
 * based on the host platform.
 *
 * @tparam T The representation of the GPIO number
 * @param gpio_number The GPIO number (this is typically different from the hardware
 * pin number)
 * @param direction The direction of the GPIO pin (input or output)
 * @param initial_state The initial state of the GPIO pin (high or low)
 */
template <typename T>
std::unique_ptr<Gpio> setupGpio(const T& gpio_number, GpioDirection direction,
                                GpioState initial_state)
{
    if constexpr (PLATFORM == Platform::JETSON_NANO)
    {
        return std::make_unique<GpioSysfs>(gpio_number, direction, initial_state);
    }
    else
    {
        return std::make_unique<GpioCharDev>(gpio_number, direction, initial_state,
                                             "/dev/gpiochip4");
    }
}

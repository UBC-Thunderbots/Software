#pragma once

#include <unistd.h>

#include "software/embedded/gpio.h"

/**
 * GPIO with the character device interface
 *
 * Available on Linux kernels 5.10 and later.
 */
class GpioCharDev : public Gpio
{
   public:
    /**
     * Communicate with GPIO pins via the new GPIO character device interface:
     * https://www.kernel.org/doc/html/next/userspace-api/gpio/chardev.html
     *
     * @param gpio_number The gpio number
     * @param direction The direction of the gpio
     * @param state The initial state of the gpio
     * @param char_dev_path The path to the gpio character device
     */
    GpioCharDev(uint32_t gpio_number, GpioDirection direction, GpioState state,
                std::string char_dev_path = "/dev/gpiochip0");

    /**
     * Destructor
     */
    virtual ~GpioCharDev();

    /**
     * Set the value to the provided state
     *
     * @param state The state
     */
    void setValue(GpioState state) override;

    /**
     * Get the current state of the gpio
     */
    GpioState getValue() override;

   private:
    /**
     * Parse the GpioState enum to a number representation
     *
     * @param state The state
     * @return The number representation of the state
     */
    uint8_t parseGpioState(GpioState state);

    int gpio_fd;  // File descriptor for the gpio
};

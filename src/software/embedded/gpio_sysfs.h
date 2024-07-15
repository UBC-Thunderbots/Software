#pragma once

#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <fstream>
#include <iostream>

#include "software/embedded/gpio.h"

/**
 * GPIO with the sysfs interface
 *
 * Deprecated in Linux kernels 4.8 and later but still available on some systems.
 */
class GpioSysfs : public Gpio
{
   public:
    /*
     * GPIO Sysfs Wrapper
     *
     * See https://www.kernel.org/doc/Documentation/gpio/sysfs.txt
     *
     * @param gpio The gpio to setup
     * @param direction The direction to configure this gpio in
     * @param initial_state The initial GpioState of the pin
     */
    GpioSysfs(const std::string& gpio_number, GpioDirection direction,
              GpioState initial_state);

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
    std::string gpio_number_;
};

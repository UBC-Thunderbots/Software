#include "software/jetson_nano/gpio.h"

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <string>

#include "software/logger/logger.h"

GPIO::GPIO(std::string gpio_number, GpioDirection direction, GpioState initial_state)
{
    // Setup the provided GPIO pin
    gpio_number_ = gpio_number;

    auto export_gpio_fs = std::ofstream("/sys/class/gpio/export");
    export_gpio_fs << gpio_number;
    export_gpio_fs.close();

    auto set_direction_fs =
        std::ofstream("/sys/class/gpio/gpio" + gpio_number + "/direction");

    switch (direction)
    {
        case GpioDirection::OUTPUT:
        {
            set_direction_fs << "out";
            break;
        }
        case GpioDirection::INPUT:
        {
            set_direction_fs << "in";
            break;
        }
    }

    set_direction_fs.close();
    setValue(initial_state);

    LOG(DEBUG) << "GPIO " << gpio_number_ << " online";
}

void GPIO::setValue(GpioState state)
{
    std::ofstream gpio_fs("/sys/class/gpio/gpio" + gpio_number_ + "/value");

    CHECK(gpio_fs.is_open()) << "Could not set GPIO pin";

    switch (state)
    {
        case GpioState::HIGH:
        {
            gpio_fs << "1";
            break;
        }
        case GpioState::LOW:
        {
            gpio_fs << "0";
            break;
        }
    }

    if (gpio_fs.is_open())
    {
        gpio_fs.close();
    }
}

GpioState GPIO::getValue()
{
    std::ifstream gpio_fs("/sys/class/gpio/gpio" + gpio_number_ + "/value");
    std::string level;

    CHECK(gpio_fs.is_open()) << "Could not read GPIO pin";
    std::getline(gpio_fs, level);

    if (level.compare("0") == 0)
    {
        return GpioState::LOW;
    }
    else
    {
        return GpioState::HIGH;
    }
}

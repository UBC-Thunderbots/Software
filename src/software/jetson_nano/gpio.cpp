#include "software/jetson_nano/gpio.h"

#include <unistd.h>

#include "software/logger/logger.h"

GPIO::GPIO(std::string gpio)
{
    // Setup the provided GPIO pin
    gpio_number_                 = gpio;
    FILE* export_file_descriptor = fopen("/sys/class/gpio/export", "w");

    if (export_file_descriptor == NULL)
    {
        LOG(FATAL) << "Unable to open /sys/class/gpio/export";
    }

    auto out = fwrite(gpio.c_str(), gpio.size(), 1, export_file_descriptor);

    if (out != 1)
    {
        LOG(FATAL) << "Error writing to /sys/class/gpio/export";
    }

    fclose(export_file_descriptor);

    FILE* direction_file_descriptor =
        fopen(("/sys/class/gpio/gpio" + gpio + "/direction").c_str(), "w");

    if (direction_file_descriptor == NULL)
    {
        LOG(FATAL) << "Unable to open /sys/class/gpio" + gpio + "/direction";
    }

    out = fwrite("out", 1, 3, direction_file_descriptor);

    if (out != 3)
    {
        LOG(FATAL) << "Error writing to /sys/class/gpio" + gpio + "/direction";
    }

    fclose(direction_file_descriptor);

    gpio_file_descriptor_ =
        fopen(("/sys/class/gpio/gpio" + gpio + "/value").c_str(), "r+");

    if (gpio_file_descriptor_ == NULL)
    {
        LOG(FATAL) << "Couldn't open GPIO pin";
    }

    LOG(DEBUG) << "GPIO " << gpio_number_ << " online";

}

GPIO::~GPIO()
{
    fclose(gpio_file_descriptor_);
}

void GPIO::setValue(GpioState state)
{
    fseek(gpio_file_descriptor_, 0, SEEK_SET);
    LOG(DEBUG) << "setting " << gpio_number_ << " to " << state;
    switch (state)
    {
        case GpioState::HIGH:
        {
            if (fwrite("1", 1, 5, gpio_file_descriptor_) != 5)
            {
                LOG(WARNING) << "Failed to set GPIO high " << gpio_number_;
            }
            current_state_ = state;
            break;
        }
        case GpioState::LOW:
        {
            if (fwrite("0", 1, 5, gpio_file_descriptor_) != 5)
            {
                LOG(WARNING) << "Failed to set GPIO low " << gpio_number_;
            }
            current_state_ = state;
            break;
        }
    }

    // TODO confirm with elec
    usleep(1000000);
}

GpioState GPIO::getValue()
{
    fseek(gpio_file_descriptor_, 0, SEEK_SET);
    int val = fgetc(gpio_file_descriptor_);

    if (val == '0')
    {
        return GpioState::LOW;
    }
    else
    {
        return GpioState::HIGH;
    }
}

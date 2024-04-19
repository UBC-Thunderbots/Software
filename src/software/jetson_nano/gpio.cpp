#include "software/jetson_nano/gpio.h"

#include "software/logger/logger.h"

#include <linux/gpio.h>

Gpio::Gpio(int gpio_number, GpioDirection direction, GpioState initial_state, std::string char_dev_path)
{
    int fd = open(char_dev_path.c_str(), O_RDONLY);
    if (fd < 0)
    {
        LOG(FATAL) << "Could not open GPIO chip device";
    }

    struct gpiohandle_request req;
    req.lineoffsets[0] = gpio_number;
    req.lines = 1;

    switch (direction)
    {
        case GpioDirection::OUTPUT:
        {
            req.flags = GPIOHANDLE_REQUEST_OUTPUT;
            break;
        }
        case GpioDirection::INPUT:
        {
            req.flags = GPIOHANDLE_REQUEST_INPUT;
            break;
        }
        default:
        {
            LOG(FATAL) << "Invalid GPIO direction: " << direction;
        } 
    }

    req.default_values[0] = parseGpioState(initial_state);

    int ret = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req);
    if (ret < 0)
    {
        LOG(FATAL) << "Could not get GPIO line handle for GPIO " << gpio_number;
    }
    close(fd);

    gpio_fd = req.fd;

    LOG(DEBUG) << "GPIO " << gpio_number << " online";
}

void Gpio::setValue(GpioState state)
{
    struct gpiohandle_data data;
    data.values[0] = parseGpioState(state);

    int ret = ioctl(gpio_fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
    if (ret < 0)
    {
        LOG(FATAL) << "Could not set GPIO value";
    }
}

Gpio::~Gpio()
{
    close(gpio_fd);
}

GpioState Gpio::getValue()
{
    struct gpiohandle_data data;
    int ret = ioctl(gpio_fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data);

    if (ret < 0)
    {
        LOG(FATAL) << "Could not get GPIO value";
    }

    switch (data.values[0])
    {
        case 0:
        {
            return GpioState::LOW;
        }
        case 1:
        {
            return GpioState::HIGH;
        }
    }

    LOG(FATAL) << "Unable to parse GPIO value";
    return GpioState::LOW;
}

uint8_t Gpio::parseGpioState(GpioState gpio_state)
{
    switch (gpio_state)
    {
        case GpioState::LOW:
        {
            return 0;
        }
        case GpioState::HIGH:
        {
            return 1;
        }
    }

    LOG(FATAL) << "Invalid GPIO state: " << gpio_state;
    return -1;
}

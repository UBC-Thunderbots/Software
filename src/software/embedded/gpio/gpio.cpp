#include "software/embedded/gpio/gpio.h"

#include <chrono>
#include <thread>

bool Gpio::pollValue(GpioState state, std::chrono::milliseconds timeout_ms)
{
    const auto start_time = std::chrono::system_clock::now();
    while (getValue() != state)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(100));

        const auto current_time = std::chrono::system_clock::now();
        const auto elapsed_time = current_time - start_time;
        if (elapsed_time > timeout_ms)
        {
            return false;
        }
    }
    return true;
}
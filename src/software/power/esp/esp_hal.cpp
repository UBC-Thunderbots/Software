#include "esp_hal.h"

#include <cstdlib>

EspHal* volatile EspHal::instance_ = nullptr;

EspHal::EspHal()
{
    if (instance_ != nullptr)
        abort();
    instance_ = this;
    initializeChicker();
    initializeCharger();
    initializeDribbler();
    initializePowerMonitor();

    // Pins are already at safe levels. Fail at startup rather than run without
    // hardware-timed pulse shutdown.
    if (pulse_timer_ == nullptr)
        abort();
}

EspHal::~EspHal()
{
    stopChickerPulse();
    timerEnd(pulse_timer_);
    setCapacitorCharging(false);
    setDribblerPwm(0);
    if (instance_ == this)
        instance_ = nullptr;
}

uint32_t EspHal::micros() const
{
    return ::micros();
}

#include <Wire.h>

#include "esp_hal.h"
#include "pins.h"

namespace
{
constexpr double CURRENT_SENSE_RESISTANCE_OHMS = 0.002;
}  // namespace

void Esp32Hal::initializePowerMonitor()
{
    // See https://github.com/espressif/arduino-esp32/issues/6616
    Wire.begin(PM_SDA, PM_SCL);
    monitor_.init(LTC4151::L, LTC4151::L);
}

float Esp32Hal::getBatteryVoltage()
{
    return static_cast<float>(monitor_.getInputVoltage());
}

float Esp32Hal::getCurrentDrawAmp()
{
    return static_cast<float>(monitor_.getLoadCurrent(CURRENT_SENSE_RESISTANCE_OHMS) /
                              1000.0);
}

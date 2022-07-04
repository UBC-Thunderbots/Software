#include "power_monitor.h"

#include <Wire.h>

PowerMonitor::PowerMonitor()
{
    // See https://github.com/espressif/arduino-esp32/issues/6616
    Wire.begin(static_cast<int>(PM_SDA), static_cast<int>(PM_SCL));

    monitor = std::make_shared<LTC4151>();
    monitor->init(LTC4151::L, LTC4151::L);
}

float PowerMonitor::getBatteryVoltage()
{
    return static_cast<float>(monitor->getInputVoltage());
}

float PowerMonitor::getCurrentDrawAmp()
{
    return static_cast<float>(monitor->getLoadCurrent(RESISTANCE_OHMS) / 1000.0);
}

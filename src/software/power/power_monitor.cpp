#include "power_monitor.h"

PowerMonitor::PowerMonitor()
{
    monitor = std::make_shared<LTC4151>();
    monitor->init(LTC4151::L, LTC4151::L);
}

float PowerMonitor::getBatteryVoltage()
{
    return static_cast<float>((monitor->getInputVoltage() / RESOLUTION) * SCALE_VOLTAGE);
}

float PowerMonitor::getCurrentDraw()
{
    return static_cast<float>(monitor->getLoadCurrent(RESISTANCE));
}

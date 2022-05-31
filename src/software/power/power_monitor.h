#pragma once

#include <LTC4151_platformio.h>

#include <memory>

#include "pins.h"

/**
 * Represents the power monitor on the power board
 */
class PowerMonitor
{
   public:
    /**
     * Creates a Power Monitor setting up part relevant firmware
     */
    PowerMonitor();
    /**
     * Returns the current battery voltage
     *
     * @return the current battery voltage
     */
    float getBatteryVoltage();
    /**
     * Returns the current current draw
     *
     * @return the current current draw
     */
    float getCurrentDraw();

   private:
    static constexpr uint16_t RESOLUTION = 4096;
    static constexpr float SCALE_VOLTAGE = 3.3;
    static constexpr double RESISTANCE = 0.002;
    std::shared_ptr<LTC4151> monitor;
};

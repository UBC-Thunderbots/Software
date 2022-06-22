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
     * Returns the current battery voltage in volts
     *
     * @return the current battery voltage in volts
     */
    float getBatteryVoltage();
    /**
     * Returns the current current draw in amps
     *
     * @return the current current draw in amps
     */
    float getCurrentDrawAmp();

   private:
    static constexpr double RESISTANCE_OHMS = 0.002;
    std::shared_ptr<LTC4151> monitor;
};

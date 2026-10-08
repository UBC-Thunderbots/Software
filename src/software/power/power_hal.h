#pragma once

#include <cstdint>

/**
 * An abstract interface for the power board hardware.
 */
class PowerHAL
{
   public:
    virtual ~PowerHAL() = default;

    // Chicker
    /**
     * Fires the kicker.
     *
     * @param pulse_width kicker pulse width in microseconds
     */
    virtual void kick(uint32_t pulse_width) = 0;
    /**
     * Fires the chipper.
     *
     * @param pulse_width chipper pulse width in microseconds
     */
    virtual void chip(uint32_t pulse_width) = 0;

    // Chicker break beam
    /**
     * @return true if the break beam is tripped, false otherwise
     */
    virtual bool getBreakBeamTripped() = 0;

    // Dribbler
    /**
     * @param speed_rpm target dribbler speed in RPM
     */
    virtual void setDribblerSpeed(uint32_t speed_rpm) = 0;
    /**
     * Advances the dribbler ramp and applies the resulting PWM output.
     */
    virtual void updateDribbler() = 0;

    // Charger
    /**
     * Sets whether the capacitors should be charging.
     *
     * @param pin_state true to begin charging the capacitors, false otherwise
     */
    virtual void setCapacitorPin(bool pin_state) = 0;
    /**
     * Updates the capacitor charger and recharges if necessary.
     */
    virtual void updateCharger() = 0;
    /**
     * @return the capacitor voltage in volts
     */
    virtual float getCapacitorVoltage() = 0;

    // Power monitor
    /**
     * @return the battery voltage in volts
     */
    virtual float getBatteryVoltage() = 0;
    /**
     * @return the current draw in amps
     */
    virtual float getCurrentDrawAmp() = 0;
};

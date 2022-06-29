#pragma once

#include <pins.h>

/*
 * Represents the charger on the power board
 */
class Charger
{
   public:
    /**
     * Creates a Charger setting up pins and attaching interrupts.
     */
    Charger();
    /**
     * Sets the charge pin to HIGH to begin charging the capacitors.
     * Note: we run in regulation mode meaning the capacitors will recharge continuously while charge is HIGH
     *
     */
    static void chargeCapacitors();
    /**
     * Returns the voltage of the capacitors
     *
     * @return voltage of capacitors
     */
    float getCapacitorVoltage();
    /**
     * Returns the status of the flyback fault
     *
     * @return whether the flyback fault is tripped or not
     */
    bool getFlybackFault();

   private:
    static constexpr float VOLTAGE_DIVIDER = 1003.0 / 13.0;
    static constexpr float RESOLUTION      = 4096.0;
    static constexpr float SCALE_VOLTAGE   = 3.3;
};

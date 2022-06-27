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
     * Sets the charge pin to either HIGH to charge or LOW to discharge given operation.
     *
     */
    static void chargeCapacitors();
    static void dischargeCapacitors();
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
    static void chargeDone();
    static hw_timer_t* charge_timer;
    static void (*volatile charge_done_callback)();
    static constexpr uint32_t CHARGE_TIME_MICROSECONDS = 3 * MICROSECONDS_IN_SECOND;
    static constexpr float VOLTAGE_DIVIDER             = 1003.0 / 13.0;
    static constexpr float RESOLUTION                  = 4096.0;
    static constexpr float SCALE_VOLTAGE               = 3.3;
    static volatile bool flyback_fault;
};

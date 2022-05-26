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
     * Sets the charge mode with a given operation. Valid operations are LOW and HIGH.
     *
     * @param op operation to be set
     */
    void setChargeMode(int op);
    /**
     * Sets up a callback for when charging is done. This callback is only called once
     *
     * @param charge_done_callback callback function to set
     */
    void setChargeDoneCallbackOnce(void (*charge_done_callback)());
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
    /**
     * Isr called when charging is done. This checks the flyback status and also calls the charge_done_callback.
     */
    static void IRAM_ATTR chargeDone();
    static void (*charge_done_callback)();
    static volatile bool flyback_fault;
};

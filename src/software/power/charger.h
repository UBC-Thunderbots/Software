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
     * Sets the state of the capacitors and whether we should charge them.
     * @param pin_state HIGH to begin charging the capacitors. Otherwise, sets the charge
     * pin to LOW.
     */
    static void setCapacitorPin(bool pin_state);
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

    /**
     * Updates the capacitor charger and recharges if necessary.
     */
    void update();

   private:
    /**
     * Reads the Done pin to determine if the capacitors have finished charging.
     * The Done pin is driven LOW (below DONE_PIN_THRESHOLD_VOLTAGE) once charging
     * completes.
     *
     * @return True if the done pin is below threshold (charging done), false otherwise
     */
    bool getDonePinState();

    /**
     * Called on the falling edge of the Charge Done pin (CHRG_DONE) to flag that the
     * capacitors need to be recharged.
     */
    static void updateCHRGDoneISR();

    static volatile bool CHRG_DONE_STATE;
    static constexpr float VOLTAGE_DIVIDER            = 1003.0 / 13.0;
    static constexpr float RESOLUTION                 = 4096.0;
    static constexpr float SCALE_VOLTAGE              = 3.3;
    static constexpr float DONE_PIN_THRESHOLD_VOLTAGE = 1.0;
};

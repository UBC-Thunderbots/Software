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
     * Updates the capacitor charger and recharges if necessary.
     */
    void update();

   private:
    /**
     * Reads CHRG_DONE with the ADC to confirm the capacitors have finished charging.
     *
     * @return True if CHRG_DONE is below threshold (charging done), false otherwise
     */
    static bool getDonePinState();

    /**
     * Called on the falling edge of CHRG_DONE.
     */
    static void updateCHRGDoneISR();

    static volatile bool charge_done_pending;
    static constexpr float VOLTAGE_DIVIDER                  = 1003.0 / 13.0;
    static constexpr float MAX_CHARGE_DURATION_MILLISECONDS = 2000;
    static constexpr float RESOLUTION                       = 4096.0;
    static constexpr float SCALE_VOLTAGE                    = 3.3;
    static constexpr float DONE_PIN_THRESHOLD_VOLTAGE       = 1.0;
};

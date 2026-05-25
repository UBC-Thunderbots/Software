#pragma once

#include <Arduino.h>

/**
 * Represents the chicker on the power board
 */
class Chicker
{
   public:
    /**
     * Creates a Chicker setting up relevant pins
     */
    Chicker();

    /**
     * Triggers a kick with the specified pulse width.
     *
     * The pulse width determines how long the kicker solenoid is activated.
     * If the system is on cooldown, the kick will be ignored.
     *
     * @param kick_pulse_width duration of the kick pulse in microseconds
     */
    static void kick(uint32_t kick_pulse_width);

    /**
     * Triggers a chip with the specified pulse width.
     *
     * The pulse width determines how long the chipper solenoid is activated.
     * If the system is on cooldown, the chip will be ignored.
     *
     * @param chip_pulse_width duration of the chip pulse in microseconds
     */
    static void chip(uint32_t chip_pulse_width);

    /**
     * Triggers a kick if the break beam is tripped.
     *
     * @param kick_pulse_width duration of the kick pulse in microseconds
     */
    static void autokick(uint32_t kick_pulse_width);

    /**
     * Triggers a chip if the break beam is tripped.
     *
     * @param chip_pulse_width duration of the chip pulse in microseconds
     */
    static void autochip(uint32_t chip_pulse_width);

    /**
     * Get the current status of whether the break beam is tripped
     *
     * @return true if the break beam is currently tripped, false otherwise
     */
    static bool getBreakBeamTripped();

   private:
    /**
     * Along with stopPulse creates a square wave to drive the chicker
     * @param duration pulse width duration in microseconds
     * @param pin the pin the send the pulse to
     */
    static void oneShotPulse(int duration, int pin);

    static void setChargeHigh();

    /**
     * Called on a pulse_timer to bring the CHIPPER/KICKER pin low
     */
    static void stopPulse();
    static void offCooldown();

    static hw_timer_t* pulse_timer;
    static hw_timer_t* cooldown_timer;
    static hw_timer_t* charge_timer;

    static volatile bool on_cooldown;

    static constexpr unsigned int CHRG_DONE_PIN  = 25;
    static constexpr unsigned int CHRG_PIN       = 26;
    static constexpr unsigned int KICKER_PIN     = 33;
    static constexpr unsigned int CHIPPER_PIN    = 32;
    static constexpr unsigned int BREAK_BEAM_PIN = 37;

    static constexpr unsigned int COOLDOWN_MICROSECONDS = 3000000;
    static constexpr unsigned int CHARGE_MICROSECONDS   = 100000;

    static constexpr unsigned int CHICKER_PULSE_TIMER    = 0;
    static constexpr unsigned int CHICKER_COOLDOWN_TIMER = 3;
    static constexpr unsigned int CHARGE_TIMER           = 1;
};

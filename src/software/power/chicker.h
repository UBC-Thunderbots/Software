#pragma once

#include <Arduino.h>
#include <pins.h>

/**
 * Represents the chicker on the power board
 */
class Chicker
{
   public:
    /**
     * Creates a Chicker setting up relevant pins and attaching interrupts
     */
    Chicker();
    /**
     * Sets the action of the chicker. Arguments can not be passed to isr's so these
     * need to be set before calling kick/chip
     */
    static void kick(uint32_t kick_pulse_width);
    static void chip(uint32_t chip_pulse_width);
    /**
     * Attaches an interrupt on the BREAK_BEAM_PIN to kick/chip.
     * kick/chip will only be triggered once
     */
    static void autokick(uint32_t kick_pulse_width);
    static void autochip(uint32_t chip_pulse_width);
    /**
     * Get the current status of whether the break beam was tripped or not
     * This is reset before every kick/chip
     *
     * @return whether the break beam has been tripped for the current action
     */
    static bool getBreakBeamTripped();

   private:
    /**
     * Along with stopPulse creates a square wave to drive the chicker
     * @param duration pulse width duration in microseconds
     * @param pin the pin the send the pulse to
     */
    static void oneShotPulse(int duration, int pin);
    /**
     * Called on a pulse_timer to bring the CHIPPER/KICKER pin low
     */
    static void stopPulse();
    static void offCooldown();

    static hw_timer_t* pulse_timer;
    static hw_timer_t* cooldown_timer;

    static volatile bool on_cooldown;
    static constexpr int COOLDOWN_MICROSECONDS = 3 * MICROSECONDS_IN_SECOND;
};

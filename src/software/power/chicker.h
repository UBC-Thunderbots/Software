#pragma once

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
    static void kick();
    static void chip();
    /**
     * Attaches an interrupt on the BREAK_BEAM_PIN to kick/chip.
     * kick/chip will only be triggered once
     */
    static void autokick();
    static void autochip();
    /**
     * Get the current status of whether the break beam was tripped or not
     * This is reset before every kick/chip
     *
     * @return whether the break beam has been tripped for the current action
     */
    static bool getBreakBeamTripped();
    /**
     * Sets the kick/chip speed/distance. Arguments can not be passed to isr's so these
     * need to be set before calling kick/chip
     * @param kick_speed_m_per_s/chip_distance_meters speed/distance to kick/chip
     */
    static void setKickSpeedMPerS(float kick_speed_m_per_s);
    static void setChipDistanceMeters(float chip_distance_meters);

   private:
    /**
     * Converts given speed/distance into pulse width duration
     *
     * @param speed_m_per_s/distance_meters speed/distance to convert to pulse width
     * duration
     * @return pulse width duration in microseconds
     */
    static int speedToPulseWidth(float speed_m_per_s);
    static int distanceToPulseWidth(float distance_meters);
    /**
     * Along with stopPulse creates a square wave to drive the chicker
     * @param duration pulse width duration in microseconds
     * @param pin the pin the send the pulse to
     */
    static void oneShotPulse(int duration, int pin);
    /**
     * Isr called when the BREAK_BEAM_PIN is tripped. Performs a kick/chip action if the
     * break beam wasn't previously triggered
     */
    static void autoKickISR();
    static void autoChipISR();
    /**
     * Called on a pulse_timer to bring the CHIPPER/KICKER pin low
     */
    static void stopPulse();
    static hw_timer_t* pulse_timer;
    static volatile bool breakbeam_tripped;

    static float kick_speed_m_per_s;
    static float chip_distance_meters;
};

#pragma once

#include "pins.h"

/**
 * Represents the geneva motor on the power board
 */
class Geneva
{
   public:
    /**
     * Creates a Geneva setting up relevant pins and attaching interrupts
     */
    Geneva();
    /**
     * Returns the current angle of the geneva motor
     *
     * @returns the current angle of the geneva motor
     */
    float getCurrentAngle();
    /**
     * Sets the angle of the geneva motor.
     * Also attaches a timer interrupt to asynchronously wait a fixed amount of time
     * (proportional to how much the geneva motor needs to rotate) before running the
     * given isr
     *
     * @param angle_deg angle to set the geneva motor to
     * @param isr isr to run once geneva motor is in place
     */
    void setAngle(float angle_deg, void (*isr)());

   private:
    /**
     * Helper called when the geneva motor is set to the right position
     *
     * @param isr isr to run once geneva motor is in place
     */
    void performWhenDone(void (*isr)());
    static hw_timer_t* timer;
};

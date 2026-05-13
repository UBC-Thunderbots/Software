#pragma once

#include <pins.h>

/**
 * Represents the dribbler on the power board
 */
class Dribbler 
{
    public:
    /**
     * Creates a dribbler, setting up pins.
     */
    Dribbler();

    /**
     * Sets the dribbler to the given duty cycle from 0 (always off) to 255 (always on)
     */
    static void dribble(uint32_t speed);
}
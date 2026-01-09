#pragma once

#include "software/util/make_enum/make_enum.hpp"

#include <chrono>

MAKE_ENUM(GpioState, LOW, HIGH);
MAKE_ENUM(GpioDirection, INPUT, OUTPUT);

/**
 * An abstract interface for interacting with General Purpose Input/Output (GPIO) pins
 */
class Gpio
{
   public:
    /**
     * Set the value to the provided state
     *
     * @param state The state
     */
    virtual void setValue(GpioState state) = 0;

    /**
     * Get the current state of the gpio
     */
    virtual GpioState getValue() = 0;

    /**
     * Polls the GPIO until it reaches the requested state or a timeout occurs
     *
     * @param state The GPIO state to wait for
     * @param timeout_ms Maximum wait duration in milliseconds
     *
     * @return true if the GPIO reached the requested state within the timeout,
     *         false if timed out before the GPIO reached the requested state
     */
    bool pollValue(GpioState state, std::chrono::milliseconds timeout_ms);
};

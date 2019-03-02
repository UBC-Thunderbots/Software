#pragma once

#include "time.h"

/**
 * A simple Duration class built around doubles. It is meant to represent some duration
 * of time
 */
class Duration : public Time
{
public:
    /**
     * Creates a new Duration value from a value in seconds.
     * @param seconds A value in seconds, from which to create the Duration
     * @return A Duration created from the given value
     */
    static const Duration fromSeconds(double seconds);

    /**
     * Creates a new Duration value from a value in milliseconds
     * @param milliseconds A value in milliseconds, from which to create the
     * Duration
     * @return A Duration created from the given value
     */
    static const Duration fromMilliseconds(double milliseconds);

    /**
     * Defines the addition operator for Durations. Allows Durations to be added to
     * Durations
     *
     * @param duration the duration to add to this Duration
     * @return A new Duration with the given Duration added to this Duration
     */
    Duration operator+(const Duration& duration) const;

    /**
     * Defines the subtraction operator for Durations. Allows Durations to be subtracted
     * from Durations
     *
     * @param duration the Duration to subtract from this Duration
     * @return A new Duration with the given Duration subtracted from this current
     * Duration
     */
    Duration operator-(const Duration& duration) const;

private:
    /**
     * Constructs a Duration value from a value in seconds.
     * @param timestamp_seconds A value >= 0.0, in seconds, from which to create the
     * timestamp
     */
    explicit Duration(double timestamp_seconds);
};

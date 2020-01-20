#pragma once

#include <iostream>

#include "software/time/time.h"

/**
 * A simple Duration class built around doubles. It is meant to represent some duration
 * of time
 */
class Duration : public Time
{
   public:
    /**
     * The default constructor for a Duration. Creates a Duration of length 0
     */
    Duration();

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
     * Compares Durations for equality. Durations are considered equal if their values
     * in seconds are within EPSILON from one another.
     *
     * @param other the Duration to compare with for equality
     * @return true if the Durations are equal and false otherwise
     */
    bool operator==(const Duration& other) const;

    /**
     * Compares Durations for inequality
     *
     * @param other the Duration to compare with for inequality
     * @return true if the Durations are not equal, and false otherwise
     */
    bool operator!=(const Duration& other) const;

    /**
     * Defines the "less than" operator. Returns true if this Duration is strictly less
     * than (and not equal to) the other Duration
     *
     * @param other the Duration to compare with
     * @return true if this Duration is strictly less than (and not equal to) the other
     * Duration, and false otherwise
     */
    bool operator<(const Duration& other) const;

    /**
     * Defines the "less than or equal to" operator. Returns true if this Duration is
     * less than or equal to the other Duration
     *
     * @param other the Duration to compare with
     * @return true if this Duration is less than or equal to the other Duration, and
     * false otherwise
     */
    bool operator<=(const Duration& other) const;

    /**
     * Defines the "greater than" operator. Returns true if this Duration is strictly
     * greater than (and not equal to) the other Duration
     *
     * @param other the Duration to compare with
     * @return true if this Duration is strictly greater than (and not equal to) the
     * other Duration, and false otherwise
     */
    bool operator>(const Duration& other) const;

    /**
     * Defines the "greater than or equal to" operator. Returns true if this Duration
     * is greater than or equal to the other Duration
     *
     * @param other the Duration to compare with
     * @return true if this Duration is greater than or equal to the other Duration, and
     * false otherwise
     */
    bool operator>=(const Duration& other) const;

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

    /**
     * Implements the << operator for printing
     *
     * @param ostream The stream to print to
     * @param time The time to print
     *
     * @return The output stream with the string representation of the class appended
     */
    friend std::ostream& operator<<(std::ostream& output_stream,
                                    const Duration& duration);

   private:
    /**
     * Constructs a Duration value from a value in seconds.
     * @param duration_seconds A value >= 0.0, in seconds, from which to create the
     *                         Duration
     */
    explicit Duration(double duration_seconds);
};

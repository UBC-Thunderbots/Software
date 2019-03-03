#pragma once

#include "duration.h"
#include "time.h"

/**
 * A simple Timestamp class built around doubles. This Timestamp is intended to represent
 * the t_capture timestamps we receive from the SSL Vision system. These t_capture values
 * are monotonic (meaning they are always positive and always increase), and are relative
 * to the "epoch time" defined by SSL Vision. This "epoch" is when SSL Vision starts up
 * and begins streaming data. Therefore, these timestamps are not absolute "wall clock"
 * time, but points in time relative to when the SSL Vision program started. They can and
 * should be used to timestamp all data received from SSL Vision and propagated throughout
 * the system in order to calculate time differences (durations), velocities, and other
 * time-dependent values.
 */
class Timestamp : public Time
{
   public:
    /**
     * The default constructor for a Timestamp. Creates a Timestamp at time 0
     */
    Timestamp();

    /**
     * Creates a new Timestamp value from a value in seconds.
     * @param seconds A value >= 0.0, in seconds, from which to create the Timestamp
     * @throws std::invalid_argument if the given value is < 0.0
     * @return A Timestamp created from the given value
     */
    static const Timestamp fromSeconds(double seconds);

    /**
     * Creates a new Timestamp value from a value in milliseconds
     * @param milliseconds A value >= 0.0, in milliseconds, from which to create the
     * Timestamp
     * @throws std::invalid_argument if the given value is < 0.0
     * @return A Timestamp created from the given value
     */
    static const Timestamp fromMilliseconds(double milliseconds);

    /**
     * Defines the addition operator for Timestamps. Allows Durations to be added to
     * Timestamps
     *
     * @param duration the Duration to add to this Timestamp
     * @return A new Timestamp with the given Duration added to this Timestamp
     */
    Timestamp operator+(const Duration& duration) const;

    /**
     * Defines the subtraction operator for Timestamps. Allows Durations to be subtracted
     * from Timestamps
     *
     * @param duration the Duration to subtract from this Timestamp
     * @return A new Timestamp with the given Duration subtracted from to this Timestamp
     */
    Timestamp operator-(const Duration& duration) const;

    /**
     * Defines the subtraction operator for Timestamps. Allows Timestamps to be subtracted
     * from Timestamps
     *
     * @param timestamp The Timestamp to subtract from this Timestamp
     * @return A Duration that is the difference in time between the two timestamps
     */
    Duration operator-(const Timestamp& timestamp) const;

   private:
    /**
     * Constructs a Timestamp value from a value in seconds.
     * @param timestamp_seconds A value >= 0.0, in seconds, from which to create the
     * timestamp
     * @throws std::invalid_argument if the provided value is < 0.0
     */
    explicit Timestamp(double timestamp_seconds);
};

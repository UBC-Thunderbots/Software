#pragma once

#include <iostream>

#include "proto/tbots_timestamp_msg.pb.h"
#include "software/time/duration.h"
#include "software/time/time.h"

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
     * Creates a new Timestamp value based on the TbotsProto::Timestamp protobuf
     * representation
     *
     * @param timestamp_proto The TbotsProto::Timestamp protobuf which this timestamp
     * should be based on
     * @throws std::invalid_argument if the given value is < 0.0
     * @return A Timestamp created from the given value
     */
    static const Timestamp fromTimestampProto(
        const TbotsProto::Timestamp& timestamp_proto);

    /**
     * Compares Timestamps for equality. Timestamps are considered equal if their values
     * in seconds are within EPSILON from one another.
     *
     * @param other the Timestamp to compare with for equality
     * @return true if the Timestamps are equal and false otherwise
     */
    bool operator==(const Timestamp& other) const;

    /**
     * Compares Timestamps for inequality
     *
     * @param other the Timestamp to compare with for inequality
     * @return true if the Timestamps are not equal, and false otherwise
     */
    bool operator!=(const Timestamp& other) const;

    /**
     * Defines the "less than" operator. Returns true if this Timestamp is strictly less
     * than (and not equal to) the other Timestamp
     *
     * @param other the Timestamp to compare with
     * @return true if this Timestamp is strictly less than (and not equal to) the other
     * Timestamp, and false otherwise
     */
    bool operator<(const Timestamp& other) const;

    /**
     * Defines the "less than or equal to" operator. Returns true if this Timestamp is
     * less than or equal to the other Timestamp
     *
     * @param other the Timestamp to compare with
     * @return true if this Timestamp is less than or equal to the other Timestamp, and
     * false otherwise
     */
    bool operator<=(const Timestamp& other) const;

    /**
     * Defines the "greater than" operator. Returns true if this Timestamp is strictly
     * greater than (and not equal to) the other Timestamp
     *
     * @param other the Timestamp to compare with
     * @return true if this Timestamp is strictly greater than (and not equal to) the
     * other Timestamp, and false otherwise
     */
    bool operator>(const Timestamp& other) const;

    /**
     * Defines the "greater than or equal to" operator. Returns true if this Timestamp
     * is greater than or equal to the other Timestamp
     *
     * @param other the Timestamp to compare with
     * @return true if this Timestamp is greater than or equal to the other Timestamp, and
     * false otherwise
     */
    bool operator>=(const Timestamp& other) const;

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

    /**
     * Implements the << operator for printing
     *
     * @param ostream The stream to print to
     * @param time The time to print
     *
     * @return The output stream with the string representation of the class appended
     */
    friend std::ostream& operator<<(std::ostream& output_stream, const Timestamp& time);

   private:
    /**
     * Constructs a Timestamp value from a value in seconds.
     * @param timestamp_seconds A value >= 0.0, in seconds, from which to create the
     * timestamp
     * @throws std::invalid_argument if the provided value is < 0.0
     */
    explicit Timestamp(double timestamp_seconds);
};

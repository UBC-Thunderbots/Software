#pragma once

// The forward declaration is required for the typedef to appear before the class
class Timestamp;

/**
 * This Timestamp class can also represent a duration in time, so we create a typedef
 * for Durations to make that distinction explicit
 * This typedef appears before the Timestamp class so that the class can return Duration
 * values
 */
typedef Timestamp Duration;

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
class Timestamp
{
   public:
    // Due to internal representation of doubles being slightly less accurate/consistent
    // with some numbers and operations, we consider timestamps that are very close
    // together to be equal (since they likely are, just possibly slightly misrepresented
    // by the system/compiler). We use this EPSILON as a threshold for comparison. 1e-15
    // was chosen as a value because doubles have about 16 consistent significant figures.
    // Comparing numbers with 15 significant figures gives us a
    // small buffer while remaining as accurate as possible.
    // http://www.cplusplus.com/forum/beginner/95128/
    static constexpr double EPSILON = 1e-15;

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
     * Returns the value of the Timestamp in seconds
     * @return the value of the Timestamp in seconds
     */
    double getSeconds() const;

    /**
     * Returns the value of the Timestamp in milliseconds
     * @return the value of the Timestamp in milliseconds
     */
    double getMilliseconds() const;

    /**
     * Compares Timestamps for equality. Timestamps are considered equal if their values
     * in seconds are within EPSILON from one another.
     *
     * @param other the Timestamp to compare with for equality
     * @return true if the timestamps are equal and false otherwise
     */
    bool operator==(const Timestamp& other) const;

    /**
     * Compares Timestamps for inequality
     *
     * @param other the Timestamp to compare with for inequality
     * @return true if the timestamps are not equal, and false otherwise
     */
    bool operator!=(const Timestamp& other) const;

    /**
     * Defines the "less than" operator. Returns true if this Timestamp is strictly less
     * than (and not equal to) the other timestamp
     *
     * @param other the Timestamp to compare with
     * @return true if this Timestamp is strictly less than (and not equal to) the other
     * timestamp, and false otherwise
     */
    bool operator<(const Timestamp& other) const;

    /**
     * Defines the "less than or equal to" operator. Returns true if this Timestamp is
     * less than or equal to the other timestamp
     *
     * @param other the Timestamp to compare with
     * @return true if this Timestamp is less than or equal to the other timestamp, and
     * false otherwise
     */
    bool operator<=(const Timestamp& other) const;

    /**
     * Defines the "greater than" operator. Returns true if this Timestamp is strictly
     * greater than (and not equal to) the other timestamp
     *
     * @param other the Timestamp to compare with
     * @return true if this Timestamp is strictly greater than (and not equal to) the
     * other timestamp, and false otherwise
     */
    bool operator>(const Timestamp& other) const;

    /**
     * Defines the "greater than or equal to" operator. Returns true if this Timestamp
     * is greater than or equal to the other timestamp
     *
     * @param other the Timestamp to compare with
     * @return true if this Timestamp is greater than or equal to the other timestamp, and
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
     * @return A new Timestamp with the given Duration subtracted from this current
     * Timestamp
     */
    Timestamp operator-(const Duration& duration) const;

   private:
    /**
     * Constructs a Timestamp value from a value in seconds.
     * @param timestamp_seconds A value >= 0.0, in seconds, from which to create the
     * timestamp
     * @throws std::invalid_argument if the provided value is < 0.0
     */
    explicit Timestamp(double timestamp_seconds);

    // The stored internal value of the timestamp, in seconds
    double timestamp_in_seconds;
};

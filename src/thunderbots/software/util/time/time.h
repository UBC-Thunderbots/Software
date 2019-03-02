#pragma once

/**
 * A simple Time class that represents some amount of Time
 */
class Time
{
   public:
    // Due to internal representation of doubles being slightly less accurate/consistent
    // with some numbers and operations, we consider Times that are very close
    // together to be equal (since they likely are, just possibly slightly misrepresented
    // by the system/compiler). We use this EPSILON as a threshold for comparison. 1e-15
    // was chosen as a value because doubles have about 16 consistent significant figures.
    // Comparing numbers with 15 significant figures gives us a
    // small buffer while remaining as accurate as possible.
    // http://www.cplusplus.com/forum/beginner/95128/
    static constexpr double EPSILON = 1e-15;

    /**
     * The default constructor for a Time. Creates a Time at time 0
     */
    Time();

    /**
     * Creates a new Time value from a value in seconds.
     * @param seconds A value in seconds, from which to create the Time
     * @return A Time created from the given value
     */
    static const Time fromSeconds(double seconds);

    /**
     * Creates a new Time value from a value in milliseconds
     * @param milliseconds A value in milliseconds, from which to create the
     * Time
     * @return A Time created from the given value
     */
    static const Time fromMilliseconds(double milliseconds);

    /**
     * Returns the value of the Time in seconds
     * @return the value of the Time in seconds
     */
    double getSeconds() const;

    /**
     * Returns the value of the Time in milliseconds
     * @return the value of the Time in milliseconds
     */
    double getMilliseconds() const;

    /**
     * Compares Times for equality. Times are considered equal if their values
     * in seconds are within EPSILON from one another.
     *
     * @param other the Time to compare with for equality
     * @return true if the Times are equal and false otherwise
     */
    bool operator==(const Time& other) const;

    /**
     * Compares Times for inequality
     *
     * @param other the Time to compare with for inequality
     * @return true if the Times are not equal, and false otherwise
     */
    bool operator!=(const Time& other) const;

    /**
     * Defines the "less than" operator. Returns true if this Time is strictly less
     * than (and not equal to) the other Time
     *
     * @param other the Time to compare with
     * @return true if this Time is strictly less than (and not equal to) the other
     * Time, and false otherwise
     */
    bool operator<(const Time& other) const;

    /**
     * Defines the "less than or equal to" operator. Returns true if this Time is
     * less than or equal to the other Time
     *
     * @param other the Time to compare with
     * @return true if this Time is less than or equal to the other Time, and
     * false otherwise
     */
    bool operator<=(const Time& other) const;

    /**
     * Defines the "greater than" operator. Returns true if this Time is strictly
     * greater than (and not equal to) the other Time
     *
     * @param other the Time to compare with
     * @return true if this Time is strictly greater than (and not equal to) the
     * other Time, and false otherwise
     */
    bool operator>(const Time& other) const;

    /**
     * Defines the "greater than or equal to" operator. Returns true if this Time
     * is greater than or equal to the other Time
     *
     * @param other the Time to compare with
     * @return true if this Time is greater than or equal to the other Time, and
     * false otherwise
     */
    bool operator>=(const Time& other) const;

    /**
     * Defines the addition operator for Times. Allows Times to be added to
     * Times
     *
     * @param time the time to add to this Time
     * @return A new Time with the given Time added to this Time
     */
    Time operator+(const Time& time) const;

    /**
     * Defines the subtraction operator for Times. Allows Times to be subtracted
     * from Times
     *
     * @param time the Time to subtract from this Time
     * @return A new Time with the given Time subtracted from this current
     * Time
     */
    Time operator-(const Time& time) const;

protected:
    /**
     * Constructs a Time value from a value in seconds.
     * @param Time_seconds A value in seconds, from which to create the
     * Time
     */
    explicit Time(double Time_seconds);

private:
    // The stored internal value of the Time, in seconds
    double time_in_seconds;
};

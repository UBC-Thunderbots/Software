#pragma once

#include <time.h>

/**
 * A simple Time class that represents some amount of Time
 *
 * This class should not be used directly, rather, you should use one of it's subclasses
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
     * Returns the value of the Time in seconds
     *
     * @return the value of the Time in seconds
     */
    double toSeconds() const;

    /**
     * Returns the value of the Time in milliseconds
     *
     * @return the value of the Time in milliseconds
     */
    double toMilliseconds() const;

    /**
     * Destructor
     *
     * We declare this virtual because no one should use this class directly, but instead
     * should use one of it's subclasses.
     *
     * Note however that we give it an implementation, so subclasses will by default not
     * be abstract.
     */
    virtual ~Time() = 0;

   protected:
    /**
     * Constructs a Time value from a value in seconds.
     *
     * @param time_seconds A value in seconds, from which to create the
     * Time
     */
    explicit Time(double time_seconds);

    // The stored internal value of the Time, in seconds
    double time_in_seconds;
};

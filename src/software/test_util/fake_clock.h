#pragma once

#include <chrono>
#include <cstdint>
#include <ratio>

/**
 * Fake clock convenient for testing purposes.
 *
 * Satisfies the TrivialClock requirements and thus can replace any standard clock
 * in the chrono library (e.g. std::chrono::steady_clock).
 */
class FakeClock {
   public:
    typedef uint64_t rep;
    typedef std::nano period;
    typedef std::chrono::duration<rep, period> duration;
    typedef std::chrono::time_point<FakeClock> time_point;
    inline static const bool is_steady = false;

    /**
     * Returns the current time point.
     *
     * @return the current time point
     */
    static time_point now() noexcept;

    /**
     * Advances the current time point by the given amount of time.
     *
     * @param time the amount of time to advance the current time point by
     */
    static void advance(duration time) noexcept;

    /**
     * Resets the current time point to the clock's epoch
     * (the origin of fake_clock::time_point)
     */
    static void reset() noexcept;

   private:
    FakeClock() = delete;
    ~FakeClock() = delete;
    FakeClock(FakeClock const&) = delete;

    static time_point now_;
};
#pragma once

#include <chrono>
#include <cstdint>
#include <ratio>

/**
 * Fake clock convenient for testing purposes.
 *
 * Tests that rely on sleeping can be non-deterministic and flaky since they
 * depend on real time to elapse. Use this clock instead, which can be explicitly
 * advanced without sleeping.
 *
 * Satisfies the TrivialClock requirements and thus can replace any standard clock
 * in the chrono library (e.g. std::chrono::steady_clock).
 */
class FakeClock
{
   public:
    using rep = uint64_t;
    using period = std::nano;
    using duration = std::chrono::duration<rep, period>;
    using time_point = std::chrono::time_point<FakeClock>;
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
    FakeClock()                 = delete;
    ~FakeClock()                = delete;
    FakeClock(FakeClock const&) = delete;

    static time_point now_;
};

#include "fake_clock.h"

FakeClock::time_point FakeClock::now_;

FakeClock::time_point FakeClock::now() noexcept
{
    return now_;
}

void FakeClock::advance(duration time) noexcept
{
    now_ += time;
}

void FakeClock::reset() noexcept
{
    now_ = FakeClock::time_point();
}

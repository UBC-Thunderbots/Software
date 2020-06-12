#include "software/time/time.h"

#include "shared/constants.h"

Time::Time() : time_in_seconds(0) {}

Time::Time(double time_seconds)
{
    this->time_in_seconds = time_seconds;
}

double Time::getSeconds() const
{
    return time_in_seconds;
}

double Time::getMilliseconds() const
{
    return time_in_seconds * MILLISECONDS_PER_SECOND;
}

Time::~Time() {}

double Time::now()
{
    const auto clock_now = std::chrono::system_clock::now();
    return static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                   clock_now.time_since_epoch())
                                   .count()) /
           1000000.0;
}

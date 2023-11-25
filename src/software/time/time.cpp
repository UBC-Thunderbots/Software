#include "software/time/time.h"

#include "shared/constants.h"

Time::Time() : time_in_seconds(0) {}

Time::Time(double time_seconds)
{
    this->time_in_seconds = time_seconds;
}

double Time::toSeconds() const
{
    return time_in_seconds;
}

double Time::toMilliseconds() const
{
    return time_in_seconds * MILLISECONDS_PER_SECOND;
}

Time::~Time() {}

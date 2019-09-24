#include "software/util/time/time.h"

#include <cmath>
#include <stdexcept>

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

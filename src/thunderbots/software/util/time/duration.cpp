/*
 * Implementation for the Duration class
 */

#include "duration.h"
#include "shared/constants.h"

Duration::Duration(double timestamp_seconds) : Time(timestamp_seconds) {}

const Duration Duration::fromSeconds(double seconds)
{
    return Duration(seconds);
}

const Duration Duration::fromMilliseconds(double milliseconds)
{
    return Duration(milliseconds * SECONDS_PER_MILLISECOND);
}

Duration Duration::operator+(const Duration &duration) const
{
    return Duration::fromSeconds(getSeconds() + duration.getSeconds());
}

Duration Duration::operator-(const Duration &duration) const
{
    return Duration::fromSeconds(getSeconds() - duration.getSeconds());
}


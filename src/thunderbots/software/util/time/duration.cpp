/*
 * Implementation for the Duration class
 */

#include "duration.h"

#include "shared/constants.h"

Duration::Duration() : Duration(0) {}

Duration::Duration(double duration_seconds) : Time(duration_seconds) {}

const Duration Duration::fromSeconds(double seconds)
{
    return Duration(seconds);
}

const Duration Duration::fromMilliseconds(double milliseconds)
{
    return Duration(milliseconds * SECONDS_PER_MILLISECOND);
}

bool Duration::operator==(const Duration &other) const
{
    return std::fabs(other.getSeconds() - getSeconds()) < EPSILON;
}

bool Duration::operator!=(const Duration &other) const
{
    return !(*this == other);
}

bool Duration::operator<(const Duration &other) const
{
    return (*this != other) && (getSeconds() < other.getSeconds());
}

bool Duration::operator>=(const Duration &other) const
{
    return !(*this < other);
}

bool Duration::operator>(const Duration &other) const
{
    return (*this != other) && (getSeconds() > other.getSeconds());
}

bool Duration::operator<=(const Duration &other) const
{
    return !(*this > other);
}

Duration Duration::operator+(const Duration &duration) const
{
    return Duration::fromSeconds(getSeconds() + duration.getSeconds());
}

Duration Duration::operator-(const Duration &duration) const
{
    return Duration::fromSeconds(getSeconds() - duration.getSeconds());
}

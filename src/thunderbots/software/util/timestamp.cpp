#include "util/timestamp.h"

#include <cmath>
#include <stdexcept>

#include "shared/constants.h"

Timestamp::Timestamp() : timestamp_in_seconds(0) {}

Timestamp::Timestamp(double timestamp_seconds)
{
    if (timestamp_seconds < 0.0)
    {
        throw std::invalid_argument(
            "Error: Timestamps cannot be created from negative values");
    }

    timestamp_in_seconds = timestamp_seconds;
}

const Timestamp Timestamp::fromSeconds(double seconds)
{
    return Timestamp(seconds);
}

const Timestamp Timestamp::fromMilliseconds(double milliseconds)
{
    return Timestamp(milliseconds * SECONDS_PER_MILLISECOND);
}

double Timestamp::getSeconds() const
{
    return timestamp_in_seconds;
}

double Timestamp::getMilliseconds() const
{
    return timestamp_in_seconds * MILLISECONDS_PER_SECOND;
}

bool Timestamp::operator==(const Timestamp &other) const
{
    return std::fabs(other.getSeconds() - getSeconds()) < EPSILON;
}

bool Timestamp::operator!=(const Timestamp &other) const
{
    return !(*this == other);
}

bool Timestamp::operator<(const Timestamp &other) const
{
    return (*this != other) && (getSeconds() < other.getSeconds());
}

bool Timestamp::operator>=(const Timestamp &other) const
{
    return !(*this < other);
}

bool Timestamp::operator>(const Timestamp &other) const
{
    return (*this != other) && (getSeconds() > other.getSeconds());
}

bool Timestamp::operator<=(const Timestamp &other) const
{
    return !(*this > other);
}

Timestamp Timestamp::operator+(const Duration &duration) const
{
    return Timestamp::fromSeconds(getSeconds() + duration.getSeconds());
}

Timestamp Timestamp::operator-(const Duration &duration) const
{
    return Timestamp::fromSeconds(getSeconds() - duration.getSeconds());
}

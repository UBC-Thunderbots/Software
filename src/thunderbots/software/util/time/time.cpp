#include "time.h"

#include <cmath>
#include <stdexcept>

#include "shared/constants.h"

Time::Time() : time_in_seconds(0) {}

Time::Time(double time_seconds)
{
    this->time_in_seconds = time_seconds;
}

const Time Time::fromSeconds(double seconds)
{
    return Time(seconds);
}

const Time Time::fromMilliseconds(double milliseconds)
{
    return Time(milliseconds * SECONDS_PER_MILLISECOND);
}

double Time::getSeconds() const
{
    return time_in_seconds;
}

double Time::getMilliseconds() const
{
    return time_in_seconds * MILLISECONDS_PER_SECOND;
}

bool Time::operator==(const Time &other) const
{
    return std::fabs(other.getSeconds() - getSeconds()) < EPSILON;
}

bool Time::operator!=(const Time &other) const
{
    return !(*this == other);
}

bool Time::operator<(const Time &other) const
{
    return (*this != other) && (getSeconds() < other.getSeconds());
}

bool Time::operator>=(const Time &other) const
{
    return !(*this < other);
}

bool Time::operator>(const Time &other) const
{
    return (*this != other) && (getSeconds() > other.getSeconds());
}

bool Time::operator<=(const Time &other) const
{
    return !(*this > other);
}

Time Time::operator+(const Time &time) const
{
    return Time::fromSeconds(getSeconds() + time.getSeconds());
}

Time Time::operator-(const Time &time) const
{
    return Time::fromSeconds(getSeconds() - time.getSeconds());
}

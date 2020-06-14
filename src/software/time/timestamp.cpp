#include "software/time/timestamp.h"

#include <iomanip>

#include "shared/constants.h"

Timestamp::Timestamp() : Timestamp(0) {}

Timestamp::Timestamp(double timestamp_seconds) : Time(timestamp_seconds)
{
    if (timestamp_seconds < 0.0)
    {
        throw std::invalid_argument(
            "Error: Timestamps cannot be created from negative values");
    }
}

const Timestamp Timestamp::fromSeconds(double seconds)
{
    return Timestamp(seconds);
}

const Timestamp Timestamp::fromMilliseconds(double milliseconds)
{
    return Timestamp(milliseconds * SECONDS_PER_MILLISECOND);
}

Timestamp Timestamp::operator+(const Duration &duration) const
{
    return Timestamp::fromSeconds(getSeconds() + duration.getSeconds());
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

Timestamp Timestamp::operator-(const Duration &duration) const
{
    return Timestamp::fromSeconds(getSeconds() - duration.getSeconds());
}

Duration Timestamp::operator-(const Timestamp &timestamp) const
{
    return Duration::fromSeconds(getSeconds() - timestamp.getSeconds());
}

std::ostream &operator<<(std::ostream &output_stream, const Timestamp &time)
{
    output_stream << std::setprecision(2) << std::fixed << time.getSeconds() << "s";

    return output_stream;
}

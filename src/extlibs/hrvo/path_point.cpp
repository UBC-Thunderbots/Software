#include "extlibs/hrvo/path_point.h"

#include <stdexcept>
#include <utility>

#include "software/geom/vector.h"

PathPoint::PathPoint(const Vector &position, const float destination_speed)
    : position_(position), speed_at_destination(destination_speed)
{
}

Vector PathPoint::getPosition() const
{
    return position_;
}

float PathPoint::getSpeed() const
{
    return speed_at_destination;
}

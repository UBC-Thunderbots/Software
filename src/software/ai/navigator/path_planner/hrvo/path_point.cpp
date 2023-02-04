#include "software/ai/navigator/path_planner/hrvo/path_point.h"


PathPoint::PathPoint(const Point &position, const double speed)
    : position(position), speed(speed)
{
}

Point PathPoint::getPosition() const
{
    return position;
}

double PathPoint::getSpeed() const
{
    return speed;
}

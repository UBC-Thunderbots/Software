#include "software/ai/navigator/path_planner/hrvo/path_point.h"


PathPoint::PathPoint(const Point &position, const double speed, Angle orientation)
    : position(position), speed(speed), orientation(orientation)
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

Angle PathPoint::getOrientation() const
{
    return orientation;
}

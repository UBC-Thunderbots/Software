#include "software/geom/shot.h"

Shot::Shot(Point point, Angle angle) : point(point), angle(angle) {}

Point Shot::getPoint() const
{
    return point;
}

Angle Shot::getAngle() const
{
    return angle;
}

#include "software/ai/evaluation/shot.h"

Shot::Shot(Point point, Angle angle) : point(point), angle(angle) {}

Point Shot::getPointToShootAt() const
{
    return point;
}

Angle Shot::getOpenAngle() const
{
    return angle;
}

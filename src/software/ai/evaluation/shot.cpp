#include "software/ai/evaluation/shot.h"

Shot::Shot(Point point, Angle angle) : point(point), angle(angle) {}

const Point &Shot::getPointToShootAt() const
{
    return point;
}

const Angle &Shot::getOpenAngle() const
{
    return angle;
}

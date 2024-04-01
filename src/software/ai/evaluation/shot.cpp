#include "software/ai/evaluation/shot.h"

Shot::Shot(Point origin, Point target, Angle angle)
    : origin(origin), target(target), angle(angle)
{
}

const Point &Shot::getOrigin() const
{
    return origin;
}

const Point &Shot::getPointToShootAt() const
{
    return target;
}

const Angle &Shot::getOpenAngle() const
{
    return angle;
}

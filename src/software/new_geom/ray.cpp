#include "software/new_geom/ray.h"

Ray::Ray() {}

Ray::Ray(const Point& start, const Angle& direction)
    : start_(start), direction_(direction)
{
}

Ray::Ray(const Point& start, const Vector& direction)
    : Ray(start, direction.orientation())
{
}

Point Ray::getStart() const
{
    return start_;
}

Angle Ray::getDirection() const
{
    return direction_;
}

void Ray::setStart(const Point& start)
{
    start_ = start;
}

void Ray::setDirection(const Angle& direction)
{
    direction_ = direction;
}

void Ray::setDirection(const Vector& direction)
{
    direction_ = direction.orientation();
}

void Ray::rotate(Angle angle)
{
    direction_ += angle;
}

Vector Ray::toUnitVector() const
{
    return Vector::createFromAngle(direction_);
}

#include "software/geom/ray.h"

Ray::Ray() : start_(Point(0, 0)), direction_(Angle::zero()) {}

Ray::Ray(const Point& start, const Angle& direction)
    : start_(start), direction_(direction)
{
}

Ray::Ray(const Point& start, const Vector& direction)
    : start_(start), direction_(direction.orientation())
{
}

const Point& Ray::getStart() const
{
    return start_;
}

const Angle& Ray::getDirection() const
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

void Ray::rotate(const Angle& angle)
{
    direction_ += angle;
}

Vector Ray::toUnitVector() const
{
    return Vector::createFromAngle(direction_);
}

#include "software/new_geom/ray.h"

#include "software/new_geom/util/collinear.h"

Ray::Ray() : start_(Point(0, 0)), direction_(Angle::zero()) {}

Ray::Ray(const Point& start, const Angle& direction)
    : start_(start), direction_(direction)
{
}

Ray::Ray(const Point& start, const Vector& direction)
    : start_(start), direction_(direction.orientation())
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

void Ray::rotate(const Angle& angle)
{
    direction_ += angle;
}

Vector Ray::toUnitVector() const
{
    return Vector::createFromAngle(direction_);
}

bool Ray::contains(const Point& point) const
{
    Point point_in_ray_direction = getStart() + toUnitVector();

    bool point_is_ray_start       = point == getStart();
    bool point_collinear_with_ray = collinear(point, getStart(), point_in_ray_direction);
    bool point_is_in_ray_direction =
        ((point - getStart()).normalize() - toUnitVector()).length() <
        GeomConstants::FIXED_EPSILON;
    return point_is_ray_start || (point_collinear_with_ray && point_is_in_ray_direction);
}

#include "software/new_geom/circle.h"

Circle::Circle() : origin_(Point()), radius_(0) {}

Circle::Circle(const Point &origin, double radius) : origin_(origin)
{
    setRadius(radius);
}

void Circle::setOrigin(const Point &o)
{
    origin_ = o;
}

Point Circle::getOrigin() const
{
    return origin_;
}

void Circle::setRadius(double radius)
{
    if (radius < 0)
    {
        throw std::invalid_argument("Circle radius cannot be negative, given: " +
                                    std::to_string(radius));
    }

    radius_ = radius;
}

double Circle::getRadius() const
{
    return radius_;
}

double Circle::area() const
{
    return M_PI * radius_ * radius_;
}

bool operator==(const Circle &c, const Circle &d)
{
    return (c.getOrigin() == d.getOrigin()) &&
           (std::abs(c.getRadius() - d.getRadius()) < FIXED_EPSILON);
}

bool operator!=(const Circle &c, const Circle &d)
{
    return !(c == d);
}

std::ostream &operator<<(std::ostream &os, const Circle &circle)
{
    os << "Circle at " << circle.getOrigin() << " with radius " << circle.getRadius();
    return os;
}

#include "software/geom/circle.h"

#include "software/geom/line.h"
#include "software/geom/vector.h"

Circle::Circle() : origin_(Point()), radius_(0) {}


Circle::Circle(const Point &origin, double radius) : origin_(origin), radius_(radius)
{
    if (radius < 0)
    {
        throw std::invalid_argument("Circle radius cannot be negative, given: " +
                                    std::to_string(radius));
    }
}

const Point &Circle::origin() const
{
    return origin_;
}

double Circle::radius() const
{
    return radius_;
}

double Circle::area() const
{
    return M_PI * radius_ * radius_;
}

bool operator==(const Circle &c, const Circle &d)
{
    return (c.origin() == d.origin()) &&
           (std::abs(c.radius() - d.radius()) < FIXED_EPSILON);
}

bool operator!=(const Circle &c, const Circle &d)
{
    return !(c == d);
}

std::ostream &operator<<(std::ostream &os, const Circle &circle)
{
    os << "Circle at " << circle.origin() << " with radius " << circle.radius();
    return os;
}

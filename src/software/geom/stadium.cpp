#include "software/geom/stadium.h"

Stadium::Stadium(const Segment &length, double radius) : segment_(length), radius_(radius)
{
    if (radius < 0)
    {
        throw std::invalid_argument("Stadium radius cannot be negative, given: " +
                                    std::to_string(radius));
    }
}

Stadium::Stadium(const Point &point1, const Point &point2, double radius)
    : segment_(Segment(point1, point2)), radius_(radius)
{
    if (radius < 0)
    {
        throw std::invalid_argument("Stadium radius cannot be negative, given: " +
                                    std::to_string(radius));
    }
}

Stadium::Stadium(const Point &point, const Vector &vector, double radius)
    : segment_(Segment(point, point + vector)), radius_(radius)
{
    if (radius < 0)
    {
        throw std::invalid_argument("Stadium radius cannot be negative, given: " +
                                    std::to_string(radius));
    }
}

Segment Stadium::segment() const
{
    return segment_;
}

double Stadium::radius() const
{
    return radius_;
}

Polygon Stadium::inner_rectangle() const
{
    Vector normal =
        segment_.toVector().rotate(Angle::fromDegrees(90)).normalize() * radius_;

    Point p1 = segment_.getStart() + normal;
    Point p2 = segment_.getEnd() + normal;
    Point p3 = segment_.getEnd() - normal;
    Point p4 = segment_.getStart() - normal;

    return Polygon{p1, p2, p3, p4};
}

double Stadium::area() const
{
    return radius_ * (M_PI * radius_ + 2 * segment_.length());
}

bool operator==(const Stadium &s1, const Stadium &s2)
{
    return ((s1.segment() == s2.segment() || s1.segment().reverse() == s2.segment()) &&
            std::abs(s1.radius() - s2.radius()) < FIXED_EPSILON);
}

bool operator!=(const Stadium &s1, const Stadium &s2)
{
    return !(s1 == s2);
}

std::ostream &operator<<(std::ostream &os, const Stadium &stadium)
{
    os << "Stadium starting at " << stadium.segment().getStart() << " ending at "
       << stadium.segment().getEnd() << " with radius " << stadium.radius();
    return os;
}

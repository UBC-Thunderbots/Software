#include "software/geom/stadium.h"

Stadium::Stadium() : length_(Segment()), radius_(0) {}

Stadium::Stadium(const Segment &length, double radius) : length_(length), radius_(radius)
{
    if(radius<0)
    {
        throw std::invalid_argument("Stadium radius cannot be negative, given: " + std::to_string(radius));
    }
}

Stadium::Stadium(const Point &point1, const Point &point2, double radius)
: length_(Segment(point1, point2)), radius_(radius)
{
    if(radius<0)
    {
        throw std::invalid_argument("Stadium radius cannot be negative, given: " + std::to_string(radius));
    }
}

Stadium::Stadium(const Point &point, const Vector &vector, double radius)
: length_(Segment(point, point + vector)), radius_(radius)
{
    if(radius<0)
    {
        throw std::invalid_argument("Stadium radius cannot be negative, given: " + std::to_string(radius));
    }
}

Segment Stadium::length() const
{
    return length_;
}

double Stadium::radius() const
{
    return radius_;
}

double Stadium::area() const
{
    return radius_ * (M_PI * radius_ + 2 * length_.length());
}

double Stadium::x_span() const
{
    return 2 * radius_ + fabs(length_.toVector().x());
}

double Stadium::y_span() const
{
    return 2 * radius_ + fabs(length_.toVector().y());
}

bool operator==(const Stadium &s1, const Stadium &s2)
{
    return ((s1.length() == s2.length() || s1.length().reverse() == s2.length()) &&
            std::abs(s1.radius() - s2.radius()) < FIXED_EPSILON);
}

bool operator!=(const Stadium &s1, const Stadium &s2)
{
    return !(s1==s2);
}

std::ostream &operator<<(std::ostream &os, const Stadium &stadium)
{
    os  << "Stadium starting at " << stadium.length().getStart() << " ending at "
        << stadium.length().getEnd() << "with radius " << stadium.radius();
    return os;
}
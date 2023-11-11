#include "stadium.h"

Stadium::Stadium() : length_(Segment()), radius_(0) {}

Stadium::Stadium(const Segment &length, double radius) : length_(length), radius_(radius) {}

Stadium::Stadium(const Point &point1, const Point &point2, double radius)
: length_(Segment(point1, point2)), radius_(radius) {}

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

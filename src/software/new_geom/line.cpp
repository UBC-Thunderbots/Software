#include "software/new_geom/line.h"

Line::Line() {}

Line::Line(const Point &first, const Point &second) : first(first), second(second)
{
    calculateLine(first, second);
}

Point Line::getFirst() const
{
    return first;
}

Point Line::getSecond() const
{
    return second;
}

double Line::getSlope() const
{
    return (second.y() - first.y()) / (second.x() - first.x());
}

void Line::setFirst(const Point &first)
{
    this->first = first;
    calculateLine(this->first, this->second);
}

void Line::setSecond(const Point &second)
{
    this->second = second;
    calculateLine(this->first, this->second);
}

Point Line::valueAt(double val) const
{
    return Point(x.valueAt(val), y.valueAt(val));
}

void Line::calculateLine(const Point &first, const Point &second)
{
    if (first != second)
    {
        double angle = std::atan2(second.y() - first.y(), second.x() - first.x());
        x            = Polynomial({first.x(), std::cos(angle)});
        y            = Polynomial({first.y(), std::sin(angle)});
    }
    else
    {
        x = Polynomial();
        y = Polynomial();
    }
}

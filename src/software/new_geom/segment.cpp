#include "software/new_geom/segment.h"

void Segment::setStart(Point o)
{
    start = o;
}

const Point& Segment::getStart() const
{
    return start;
}

void Segment::setEnd(Point o)
{
    end = o;
}

const Point& Segment::getEnd() const
{
    return end;
}

double Segment::length()
{
    return (getStart() - getEnd()).length();
}

double Segment::lengthSquared()
{
    return (getStart() - getEnd()).lengthSquared();
}

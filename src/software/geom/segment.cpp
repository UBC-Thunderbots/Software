#include "software/geom/segment.h"

Segment::Segment() {}

Segment::Segment(const Point& start, const Point& end) : start(start), end(end) {}

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

double Segment::length() const
{
    return (end - start).length();
}

double Segment::lengthSquared() const
{
    return (end - start).lengthSquared();
}

Segment Segment::reverse() const
{
    return Segment(end, start);
}

Vector Segment::toVector() const
{
    return end - start;
}

Point Segment::midPoint() const
{
    return Point(toVector() / 2 + start);
}

bool Segment::operator==(const Segment& other) const
{
    return start == other.start && end == other.end;
}

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

double Segment::length() const
{
    return (end - start).length();
};

double Segment::lengthSquared() const
{
    return (end - start).lengthSquared();
}

Segment Segment::reverse() const
{
    return Segment(end, start);
};

Vector Segment::toVector() const
{
    return end - start;
};

double Segment::slope() const
{
    return (end.y() - start.y()) / (end.x() - start.x());
};

inline bool Segment::operator==(const Segment& other) const
{
    return start == other.start && end == other.end;
}

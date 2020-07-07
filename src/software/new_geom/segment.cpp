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

inline Segment::Segment() {}

inline Segment::Segment(const Point& start, const Point& end) : start(start), end(end)
{
}

inline Segment Segment::reverse() const
{
    return Segment(end, start);
}

inline Vector Segment::toVector() const
{
    return end - start;
}

inline double Segment::length() const
{
    return (end - start).length();
}

inline double Segment::slope() const
{
    return (end.y() - start.y()) / (end.x() - start.x());
}

inline bool Segment::operator==(const Segment& other) const
{
    return start == other.start && end == other.end;
}
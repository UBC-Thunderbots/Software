#pragma once

#include "geom/point.h"
#include "line.h"

class Segment final
{
   private:
    Point start;
    Point end;

   public:
    void setSegStart(Point o)
    {
        start = o;
    }

    Point getSegStart() const
    {
        return start;
    }

    void setEnd(Point f)
    {
        end = f;
    }

    Point getEnd() const
    {
        return end;
    }

    /**
     * Creates a degenerate Segment at (0, 0)
     */
    inline explicit constexpr Segment() {}

    /**
     * Creates a Segment that starts and ends at the given points
     */
    inline explicit Segment(const Point& start, const Point& end) : start(start), end(end)
    {
    }

    /**
     * Creates a Segment that is reversed
     */
    inline Segment reverse() const
    {
        return Segment(end, start);
    }

    /**
     * Makes a Point out of this Segment.
     */
    inline Vector toVector() const
    {
        return end - start;
    }

    /**
     * Makes a line out of this Segment.
     */
    inline Line toLine()
    {
        return Line(start, end);
    }

    inline double slope() const
    {
        return (end.y() - start.y()) / (end.x() - start.x());
    }

    inline bool operator==(const Segment& other) const
    {
        return start == other.start && end == other.end;
    }
};

template <>
struct std::hash<Segment>
{
    std::size_t operator()(const Segment& seg) const
    {
        std::size_t const h1(std::hash<Point>()(seg.getSegStart()));
        std::size_t const h2(std::hash<Point>()(seg.getEnd()));
        return h1 ^ (h2 << 1);
    }
};

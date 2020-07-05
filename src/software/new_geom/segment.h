#pragma once

#include "software/new_geom/point.h"

// This is a direct copy of geom/segment.h, this is needed to unblock work on the new_geom
// shape hierarchy.
// TODO (Issue #1098): Refactor this to fit the new_geom hierarchy
class Segment final
{
   public:
    /**
     * Sets the start point of the segment to o
     *
     * @param o new start point of segment
     */
    void setSegStart(Point o)
    {
        start = o;
    }

    /**
     * Gets the start point of the segment
     *
     * @return start point of segment
     */
    const Point& getSegStart() const
    {
        return start;
    }

    /**
     * Sets the end point of the segment to o
     *
     * @param o new end point of segment
     */
    void setEnd(Point o)
    {
        end = o;
    }

    /**
     * Gets the end point of the segment
     *
     * @return end point of segment
     */
    const Point& getEnd() const
    {
        return end;
    }

    /**
     * Creates a degenerate Segment at (0, 0)
     */
    inline explicit Segment() {}

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
     * Makes a Vector out of this Segment.
     */
    inline Vector toVector() const
    {
        return end - start;
    }

    inline double length() const
    {
        return (end - start).length();
    }

    inline double slope() const
    {
        return (end.y() - start.y()) / (end.x() - start.x());
    }

    inline bool operator==(const Segment& other) const
    {
        return start == other.start && end == other.end;
    }

   private:
    Point start;
    Point end;
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

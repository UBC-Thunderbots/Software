#pragma once

#include "software/new_geom/point.h"

class Segment final
{
   public:
    /**
     * Sets the start point of the segment to o
     *
     * @param o new start point of segment
     */
    void setStart(Point o);

    /**
     * Gets the start point of the segment
     *
     * @return start point of segment
     */
    const Point& getStart() const;

    /**
     * Sets the end point of the segment to o
     *
     * @param o new end point of segment
     */
    void setEnd(Point o);

    /**
     * Gets the end point of the segment
     *
     * @return end point of segment
     */
    const Point& getEnd() const;

    // TODO function headers
    double length();

    double lengthSquared();

    /**
     * Creates a degenerate Segment at (0, 0)
     */
    inline explicit Segment(){};

    /**
     * Creates a Segment that starts and ends at the given points
     */
    inline explicit Segment(const Point& start, const Point& end){};

    /**
     * Creates a Segment that is reversed
     */
    inline Segment reverse() const
    {
        return Segment(end, start);
    };

    /**
     * Makes a Vector out of this Segment.
     */
    inline Vector toVector() const
    {
        return end - start;
    };

    inline double length() const
    {
        return (end - start).length();
    };

    inline double slope() const
    {
        return (end.y() - start.y()) / (end.x() - start.x());
    };

    inline bool operator==(const Segment& other) const
    {
        {
            return start == other.start && end == other.end;
        }
    };

   private:
    Point start;
    Point end;
};

template <>
struct std::hash<Segment>
{
    std::size_t operator()(const Segment& seg) const
    {
        std::size_t const h1(std::hash<Point>()(seg.getStart()));
        std::size_t const h2(std::hash<Point>()(seg.getEnd()));
        return h1 ^ (h2 << 1);
    }
};

#pragma once

#include "software/new_geom/point.h"
#include "software/new_geom/util/almost_equal.h"
#include "software/new_geom/util/collinear.h"

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

    /**
     * Returns true if this segment contains the given point, false otherwise.
     *
     * @param point
     *
     * @return true if this segment contains the given point, false otherwise
     */
    bool contains(const Point& point, double fixed_epsilon = GeomConstants::FIXED_EPSILON,
                  int ulps_distance = GeomConstants::ULPS_EPSILON_TEN) const
    {
        if (collinear(point, getSegStart(), getEnd()))
        {
            // If the segment and point are in a perfect vertical line, we must use Y
            // coordinate centric logic
            if (almostEqual(point.x(), getEnd().x(), fixed_epsilon, ulps_distance) &&
                almostEqual(getEnd().x(), getSegStart().x(), fixed_epsilon,
                            ulps_distance))
            {
                // Since segment and point are collinear we only need to check one of the
                // coordinates, in this case we select Y because all X values are equal
                return (point.y() <= getSegStart().y() && point.y() >= getEnd().y()) ||
                       (point.y() <= getEnd().y() && point.y() >= getSegStart().y());
            }

            // Since segment and point are collinear we only need to check one of the
            // coordinates, choose x because we know there is variance in these values
            return (point.x() <= getSegStart().x() && point.x() >= getEnd().x()) ||
                   (point.x() <= getEnd().x() && point.x() >= getSegStart().x());
        }

        return false;
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

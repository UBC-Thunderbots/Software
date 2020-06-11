#pragma once

#include <vector>

#include "software/geom/point.h"
#include "software/geom/util/almost_equal.h"

class Segment final
{
   public:
    Segment() = delete;

    /**
     * Creates a Segment that starts and ends at the given points
     *
     * @throws std::invalid_argument if start==end
     */
    explicit Segment(const Point& start, const Point& end);

    /**
     * Sets the start point of the segment to o
     *
     * @param o new start point of segment
     */
    void setStart(const Point& o);

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
    void setEnd(const Point& f);

    /**
     * Gets the end point of the segment
     *
     * @return end point of segment
     */
    const Point& getEnd() const;

    /**
     * Creates a Segment that is reversed
     *
     * @return reversed segment
     */
    const Segment& reverse() const;

    /**
     * Makes a Vector out of this Segment.
     *
     * @return vector
     */
    Vector toVector() const;

    /**
     * Gets length of the segment
     *
     * @return length of the segment
     */
    double length() const;

    bool operator==(const Segment& other) const;

    /**
     * Returns true if this segment contains the given point, false otherwise.
     *
     * @param point
     *
     * @return true if this segment contains the given point, false otherwise
     */
    bool contains(const Point& point, double fixed_epsilon = GeomConstants::FIXED_EPSILON,
                  int ulps_distance = GeomConstants::ULPS_EPSILON_TEN) const;

    /**
     * Checks if this is collinear with another segment
     *
     * @param other The other segment
     * @param fixed_epsilon the epsilon value for near zero double comparisons
     * @param ulps_epsilon the epsilon value for double comparisons based on ULPs distance
     *
     * @return true if collinear with another segment, false otherwise
     */
    bool collinear(const Segment& other,
                   double fixed_epsilon = GeomConstants::FIXED_EPSILON,
                   int ulps_distance    = GeomConstants::ULPS_EPSILON_TEN) const;

    /**
     * Determines whether the other segment intersects this segment each other
     *
     * @param other The other segment
     * @return true if the segments intersect each other, false otherwise
     */
    bool intersects(const Segment& other) const;

    /**
     * Computes the intersection of this segment with another one
     *
     * @param other The other segment
     *
     * @return  one of:
     *          - an empty vector if no intersections
     *          - a vector containing a single point of intersection
     *          - a vector containing two points representing the line segment of the
     * overlap if both segments are collinear and overlapping
     */
    std::vector<Point> intersection(const Segment& other) const;

    /**
     * Finds the closest point on this line segment to p.
     *
     * @param p The point
     *
     * @return the Point on line segment closest to p
     */
    Point closestPointOnSeg(const Point& p) const;

    /**
     * Gets mid point of the Segment, halfway between the start and end points
     *
     * @return mid point
     */
    Point midPoint() const
    {
        return Point(toVector() / 2 + start);
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
        std::size_t const h1(std::hash<Point>()(seg.getStart()));
        std::size_t const h2(std::hash<Point>()(seg.getEnd()));
        return h1 ^ (h2 << 1);
    }
};

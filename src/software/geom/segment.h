#pragma once

#include "software/geom/point.h"
#include "software/geom/vector.h"

class Segment final
{
   public:
    /**
     * Creates a degenerate Segment at (0, 0)
     */
    Segment();

    /**
     * Creates a Segment that starts and ends at the given points
     */
    Segment(const Point& start, const Point& end);

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

    /**
     * Gets the length of the segment
     *
     * @return length of segment
     */
    double length() const;

    /**
     * Gets the squared length of the segment
     *
     * @return squared length of segment
     */
    double lengthSquared() const;

    /**
     * Creates a Segment that is reversed
     *
     * @return reversed segment
     */
    Segment reverse() const;

    /**
     * Makes a Vector out of this Segment
     *
     * @return vector representation of the segment
     */
    Vector toVector() const;

    /**
     * Gets mid point of the Segment, halfway between the start and end points
     *
     * @return mid point
     */
    Point midPoint() const;

    /**
     * Compares with another segment for equality
     *
     * @param other the other segment
     *
     * @return true if segment is equal to other, and false otherwise.
     */
    bool operator==(const Segment& other) const;

   private:
    Point start;
    Point end;
};

/**
 * Adds a vector offset to a segment
 *
 * @param segment segment to add offset to
 * @param vector vector offset to offset segment by
 *
 * @return segment with start and end points added to vector
 */
Segment operator+(const Segment& segment, const Vector& vector);
Segment operator+(const Vector& vector, const Segment& segment);

/**
 * Subtracts a vector offset from a segment
 *
 * @param segment segment to subtract offset to
 * @param vector vector offset to negatively offset segment by
 *
 * @return segment with start and end points subtracted by vector
 */
Segment operator-(const Segment& segment, const Vector& vector);

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

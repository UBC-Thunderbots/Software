#pragma once

#include <vector>

#include "software/geom/segment.h"
#include "software/geom/shape.h"

/**
 * A shape composed of line segments.
 */
class Polygon : public virtual Shape
{
   public:
    Polygon() = delete;
    /**
     * Construct a polygon by drawing line segments between consecutive
     * Points, and then from the last Point to the first Point.
     * @param points Points that form a polygon
     */
    explicit Polygon(const std::vector<Point>& points);

    /**
     * Construct a polygon by drawing line segments between consecutive
     * Points, and then from the last Point to the first Point.
     * @param points Points that form a polygon
     */
    explicit Polygon(const std::initializer_list<Point>& points);

    /**
     * Returns the centroid of this polygon
     *
     * @return The centroid of this polygon
     */
    Point centroid() const;

    /**
     * Returns the Polygon expanded in all directions by the expansion_amount
     *
     * @param expansion_amount a non-negative expansion amount
     * @throw std::invalid_argument if expansion_amount is negative
     *
     * @return a Polygon expanded in all directions by the expansion amount
     */
    Polygon expand(double expansion_amount) const;

    /**
     * Returns perimeter length of the polygon
     *
     * @return perimeter length of the polygon
     */
    double perimeter() const;

    /**
     * Returns the line segments that form this polygon.
     * @return the line segments that form this polygon.
     */
    const std::vector<Segment>& getSegments() const;

    /**
     * Returns the points that form the polygon.
     * @return the points that form the polygon.
     */
    const std::vector<Point>& getPoints() const;

    /**
     * Creates a rectangular polygon that is oriented along the segment.
     * @param segment segment along which to construct the rectangular polygon
     * @param length_radius How much each end of the segment should be elongated by
     * @param width_radius How wide should the polygon be, measured from the segment
     * @return an oriented polygon along the 2 points that contains both points.
     */
    static Polygon fromSegment(const Segment& segment, double length_radius,
                               double width_radius);
    static Polygon fromSegment(const Segment& segment, double radius);

   protected:
    /**
     * Initializes the segments_ vector
     */
    void initSegments();

    std::vector<Point> points_;
    std::vector<Segment> segments_;
};

bool operator==(const Polygon& poly1, const Polygon& poly2);

bool operator!=(const Polygon& poly1, const Polygon& poly2);

/**
 * Implements the << operator for printing
 *
 * @param ostream The stream to print to
 * @param poly The Polygon to print
 *
 * @return The output stream with the string representation of the class appended
 */
std::ostream& operator<<(std::ostream& os, const Polygon& poly);

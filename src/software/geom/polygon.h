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
     * Creates a polygon that is oriented along the vector between the first and second
     * point. The polygon contains both points as "focal" points similar to an ellipse.
     * see definition for details
     * @param firstPoint the first point
     * @param secondPoint the second point.
     * @return an oriented polygon along the 2 points that contains both points.
     */
    static Polygon fromPoints(const Point& first_point, const Point& second_point);

   protected:
    /**
     * Returns the line segments that connect a list of points.
     * @return the line segments
     */
    static std::vector<Segment> initSegments(std::vector<Point> points);

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

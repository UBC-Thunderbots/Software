#pragma once

#include <vector>

#include "software/geom/segment.h"
#include "software/new_geom/ray.h"

class Polygon
{
   public:
    Polygon() = delete;
    /**
     * Construct a polygon by drawing line segments between consecutive
     * Points, and then from the last Point to the first Point.
     * @param _points Points that form a polygon
     */
    explicit Polygon(const std::vector<Point>& _points);

    /**
     * Construct a polygon by drawing line segments between consecutive
     * Points, and then from the last Point to the first Point.
     * @param _points Points that form a polygon
     */
    Polygon(const std::initializer_list<Point>& _points);


    /**
     * Returns true if the polygon contains the input point, and false
     * otherwise.
     * @param point a Point
     * @return if the point is contained within the polygon.
     */
    bool containsPoint(const Point& point) const;

    /**
     * Returns true if the segment intersects the polygon, and false otherwise.
     *
     * @param segment a line segment
     * @return true if the segment intersects the polygon, false otherwise
     */
    bool intersects(const Segment& segment) const;

    /**
     * Returns true if the ray intersects the polygon, false otherwise
     * @param ray a ray
     * @return true if the ray intersects the polygon, false otherwise
     */
    bool intersects(const Ray& ray) const;

    /**
     * Returns a vector of the line segments that form this polygon.
     * @return a vector of the line segments that form this polygon.
     */
    const std::vector<Segment>& getSegments() const;

    /**
     * Returns the points that form the polygon.
     * @return the points that form the polygon.
     */
    const std::vector<Point>& getPoints() const;

   private:
    // the line segments that form the polygon
    std::vector<Segment> segments;
    std::vector<Point> points;
};

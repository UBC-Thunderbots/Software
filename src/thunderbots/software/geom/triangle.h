#pragma once

#include <vector>

#include "geom/ray.h"
#include "geom/segment.h"

class Triangle
{
   public:
    Triangle() = delete;
    /**
     * Construct a triangle by drawing line segments between three
     * Points.
     * @param p1 first point of the triangle
     * @param p2 second point of triangle
     * @param p3 third point of triangle
     */
    Triangle(Point p1, Point p2, Point p3);

    /**
     * Construct a polygon by drawing line segments between consecutive
     * Points, and then from the last Point to the first Point.
     * @param _points Points that form a polygon
     */
    explicit Triangle(const std::vector<Point>& _points);


    /**
     * Construct a polygon by drawing line segments between consecutive
     * Points, and then from the last Point to the first Point.
     * @param _points Points that form a polygon
     */
    Triangle(const std::initializer_list<Point>& _points);

    /**
     * Returns true if the triangle contains the input point, and false
     * otherwise.
     * @param point a Point
     * @return if the point is contained within the polygon.
     */
    bool containsPoint(const Point& point) const;

    /**
     * Returns true if the segment intersects the triangle, and false otherwise.
     *
     * @param segment a line segment
     * @return true if the segment intersects the triangle, false otherwise
     */
    bool intersects(const Segment& segment) const;

    /**
     * Returns true if the ray intersects the polygon, false otherwise
     * @param ray a ray
     * @return true if the ray intersects the polygon, false otherwise
     */
    bool intersects(const Ray& ray) const;

    /**
     * Returns a vector of the line segments that form this triangle.
     * @return a vector of the line segments that form this triangle.
     */
    const std::vector<Segment>& getSegments() const;

    /**
     * Returns the points that form the triangle.
     * @return the points that form the triangle.
     */
    const std::vector<Point>& getPoints() const;

   private:
    // the line segments that form the polygon
    std::vector<Segment> segments;
    std::vector<Point> points;
};

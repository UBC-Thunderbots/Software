#pragma once

#include <vector>

#include "geom/polygon.h"
#include "geom/ray.h"
#include "geom/segment.h"

class Triangle : public Polygon
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

   private:
    // the line segments that form the polygon
    std::vector<Segment> segments;
    std::vector<Point> points;
};

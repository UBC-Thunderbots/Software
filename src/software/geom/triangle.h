#pragma once

#include "software/geom/convex_polygon.h"

/**
 * A triangle.
 */
class Triangle : public ConvexPolygon
{
   public:
    /**
     * Creates a Triangle from three points.
     *
     * @param point1 One of the triangle's corners
     * @param point2 Another of the triangle's corners
     * @param point3 One more of the triangle's corners
     */
    explicit Triangle(const Point &point1, const Point &point2, const Point &point3);

    /**
     * Returns the mean of all the points of this Triangle (also known as the center).
     *
     * @return the mean of all the points of this Triangle
     */
    Point mean() const;
};

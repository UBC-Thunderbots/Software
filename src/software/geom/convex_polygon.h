#pragma once

#include "software/geom/convex_shape.h"
#include "software/geom/polygon.h"

/**
 * A polygon that is convex (curved outwards).
 */
class ConvexPolygon : public Polygon, public ConvexShape
{
   public:
    ConvexPolygon() = delete;

    /**
     * Returns the area of this convex polygon
     *
     * @return The area of this convex polygon
     */
    double area() const override;

    /**
     * Construct a convex polygon by drawing line segments between consecutive
     * Points, and then from the last Point to the first Point.
     *
     * @param points Points that form a convex polygon
     *
     * @throws std::invalid_argument if the given points do not form a
     *                               convex polygon
     */
    explicit ConvexPolygon(const std::vector<Point>& points);

    /**
     * Construct a convex polygon by drawing line segments between consecutive
     * Points, and then from the last Point to the first Point.
     *
     * @param points Points that form a convex polygon
     *
     * @throws std::invalid_argument if the given points do not form a
     *                               convex polygon
     */
    explicit ConvexPolygon(const std::initializer_list<Point>& points);

   private:
    /**
     * Check that the points of this polygon make up a Convex polygon.
     *
     * Every vertex angle must be 180 degrees or less, and for each vertex angle a,
     * the running total of all a must equal 360.
     *
     * @return true if the points of this polygon make up a Convex polygon, false
     * otherwise
     */
    bool isConvex();
};

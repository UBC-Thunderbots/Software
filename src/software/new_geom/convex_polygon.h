#pragma once

#include "software/new_geom/convex_shape.h"
#include "software/new_geom/polygon.h"

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
     * Returns the centroid of this convex polygon
     *
     * @return The centroid of this convex polygon
     */
    Point centroid() const;

    /**
     * Returns the convex polygon expanded towards v
     * The convex polygon is split in half perpendicular to v and points on the half that
     * v is pointing are translated by v
     *
     * @return a convex polygon expanded by v
     */
    ConvexPolygon expand(const Vector& v) const;

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

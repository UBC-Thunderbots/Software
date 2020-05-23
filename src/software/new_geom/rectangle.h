#pragma once

#include "software/new_geom/convex_polygon.h"

/**
 * A rectangle is a ConvexPolygon of four Points with the invariant that two sides are
 * parallel to the x axis and two sides are parallel to the y axis
 */
class Rectangle : public ConvexPolygon
{
   public:
    /**
     * Creates a new Rectangle from two corners.
     *
     *   *--------p1/2      p1/2--------*
     *   |          |        |          |
     * p1/2---------*   or   *--------p1/2
     *
     * @param point1 One of the rectangle's corners
     *
     * @param point2 The corner diagonally-opposite to point1
     */
    explicit Rectangle(const Point &point1, const Point &point2);

    /**
     * Returns the length along the x-axis of the rectangle
     *
     * @return The length along the x-axis of the rectangle
     */
    double xLength() const;

    /**
     * Returns the length along the y-axis of the rectangle
     *
     * @return The length along the y-axis of the rectangle
     */
    double yLength() const;

    /**
     * Returns the centre of the rectangle
     *
     * @return The centre of the rectangle
     */
    Point centre() const;

    /**
     * Returns the <+x,+y> corner of the rectangle
     *
     * @return The <+x,+y> corner of the rectangle
     */
    Point posXPosYCorner() const;

    /**
     * Returns the <-x,+y> corner of the rectangle
     *
     * @return The <-x,+y> corner of the rectangle
     */
    Point negXPosYCorner() const;

    /**
     * Returns the <-x,-y> corner of the rectangle
     *
     * @return The <-x,-y> corner of the rectangle
     */
    Point negXNegYCorner() const;

    /**
     * Returns the <+x,-y> corner of the rectangle
     *
     * @return The <+x,-y> corner of the rectangle
     */
    Point posXNegYCorner() const;

    /**
     * Determines whether the given Point is contained within this Rectangle.
     *
     * @return whether the Point p is contained within this Rectangle.
     */
    bool contains(const Point &p) const override;

    /**
     * Returns the corner point of the rectangle that is the furthest from the input
     * point.
     *
     * @param p The point to test
     * @return The corner point that is furthest from the test point
     */
    Point furthestCorner(const Point &p);

    /**
     * Returns a vector from negXNegY point of the rectangle to the posXPosY point
     *
     * @return The vector representing the diagonal of the rectangle
     */
    Vector diagonal() const;

    /**
     * Attempts to move all edges of the rectangle outwards or inwards towards
     * the centre by an "amount" while maintaining the same location for the center of the
     * rectangle. The rectangle will not shrink to anything smaller than a point.
     *
     * NOTE: this is a deprecated function that will be removed in #1331 or #1332
     *
     * @param amount The amount to expand or shrink the rectangle by on all sides, can be
     * positive or negative
     *
     * @return bool Whether it was possible to expand/shrink the rectangle by
     * amount requested, rectangle remains unchanged if impossible to expand/shrink
     */
    bool inflate(double amount);

    /**
     * Returns the Rectangle expanded in the direction of v
     * To maintain the Rectangle invariant, this is done in two steps:
     * 1. The Rectangle is split in half perpendicular to v*(1,0) and points on the half
     * that v*(1,0) is pointing are translated by v*(1,0)
     * 2. The Rectangle is split in half perpendicular to v*(0,1) and points on the half
     * that v*(0,1) is pointing are translated by v*(0,1)
     *
     * @return a Rectangle expanded in the direction of v
     */
    Rectangle expand(const Vector &v) const;

    bool operator==(const Rectangle &p) const;
};

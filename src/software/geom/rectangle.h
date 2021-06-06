#pragma once

#include "software/geom/convex_polygon.h"

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
    const Point &posXPosYCorner() const;

    /**
     * Returns the <-x,+y> corner of the rectangle
     *
     * @return The <-x,+y> corner of the rectangle
     */
    const Point &negXPosYCorner() const;

    /**
     * Returns the <-x,-y> corner of the rectangle
     *
     * @return The <-x,-y> corner of the rectangle
     */
    const Point &negXNegYCorner() const;

    /**
     * Returns the <+x,-y> corner of the rectangle
     *
     * @return The <+x,-y> corner of the rectangle
     */
    const Point &posXNegYCorner() const;

    /**
     * Gets the maximum x value of the rectangle
     *
     * return max x value
     */
    double xMax() const;

    /**
     * Gets the minimum x value of the rectangle
     *
     * return min x value
     */
    double xMin() const;

    /**
     * Gets the maximum y value of the rectangle
     *
     * return max y value
     */
    double yMax() const;

    /**
     * Gets the minimum y value of the rectangle
     *
     * return min y value
     */
    double yMin() const;

    /**
     * Returns a vector from negXNegY point of the rectangle to the posXPosY point
     *
     * @return The vector representing the diagonal of the rectangle
     */
    Vector diagonal() const;

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

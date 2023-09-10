#pragma once

#include "software/geom/point.h"

/**
 * BoundingBox is an axis aligned rectangle which use used to wrap a shape.
 * This is used as a light weight alternative to the Rectangle class as
 * it does not extend the ConvexPolygon class.
 */
class BoundingBox // TODO: Add tests for this class
{
    public:
    BoundingBox() = delete;

    /**
     * Creates a new bounding box from two corners.
     *
     *   *--------p1/2      p1/2--------*
     *   |          |        |          |
     * p1/2---------*   or   *--------p1/2
     *
     * @param point1 One of the rectangle's corners
     *
     * @param point2 The corner diagonally-opposite to point1
     */
    explicit BoundingBox(const Point &point1, const Point &point2);


    /**
     * Returns the <+x,+y> corner of the bounding box
     *
     * @return The <+x,+y> corner of the bounding box
     */
    const Point &posXPosYCorner() const;

    /**
     * Returns the <-x,-y> corner of the bounding box
     *
     * @return The <-x,-y> corner of the bounding box
     */
    const Point &negXNegYCorner() const;

    /**
     * Gets the maximum x value of the bounding box
     *
     * return max x value
     */
    double xMax() const;

    /**
     * Gets the minimum x value of the bounding box
     *
     * return min x value
     */
    double xMin() const;

    /**
     * Gets the maximum y value of the bounding box
     *
     * return max y value
     */
    double yMax() const;

    /**
     * Gets the minimum y value of the bounding box
     *
     * return min y value
     */
    double yMin() const;

private:
    Point pos_x_pos_y_corner;
    Point neg_x_neg_y_corner;
};

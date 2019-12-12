#include "software/new_geom/convex_polygon.h"

/**
 * A rectangle.
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
     * Returns the area of the rectangle
     *
     * @return The area of the rectangle
     */
    double area() const override;

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
     * Returns a digit corresponding to a specific corner for pos%4. (0 for <-x,-y>, 1
     * for <-x,+y>, 2 for <+x,+y> and 3 for <+x,-y>)
     *
     * @param pos The position of the corner wanted
     *
     * @return Point A digit corresponding to position
     */
    Point operator[](unsigned int pos) const;

    /**
     * Attempts to move all edges of the rectangle outwards or inwards towards
     * the centre by an "amount" while maintaining the same location for the center of the
     * rectangle. The rectangle will not shrink to anything smaller than a point
     *
     * @param amount The amount to expand or shrink the rectangle by on all sides, can be
     * positive or negative
     *
     * @return bool Whether it was possible to expand or shrink the rectangle by
     * amount requested
     */
    bool expand(double amount);

    /**
     * Returns the corner point of the rectangle that is the furthest from the input
     * point.
     *
     * @param p The point to test
     * @return The corner point that is furthest from the test point
     */
    Point furthestCorner(Point p);

    /**
     * Returns the negXNegY point of the rectangle.
     *
     * @return The bottom left corner of the rectangle
     */
    Point minCorner() const;

    /**
     * Returns a vector from negXNegY point of the rectangle to the posXPosY point
     *
     * @return The vector representing the diagonal of the rectangle
     */
    Vector diagonal() const;

    bool operator==(const Rectangle &p) const;
};

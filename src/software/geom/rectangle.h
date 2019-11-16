#pragma once

#include <cmath>

#include "software/new_geom/point.h"

/**
 * A rectangle.
 */
class Rectangle final
{
   public:
    /**
     * Creates a new Rectangle from two corners
     *
     * @param point1 One of the rectangle's corners
     *
     * @param point2 The corner diagonally-opposite to point1
     */
    explicit Rectangle(const Point &point1, const Point &point2);

    /**
     * Creates a new Rectangle from a corner and a size
     *
     * @param neg_x_neg_y_corner The <-x,-y> corner of the rectangle
     *
     * @param xLength The xLength of the rectangle
     *
     * @param yLength The yLength of the rectangle
     */
    explicit Rectangle(const Point &neg_x_neg_y_corner, double x_length, double y_length);

    /**
     * Returns the length along the x-axis of the rectangle
     *
     * @return The length along the x-axis of the rectangle
     */
    constexpr double xLength() const;

    /**
     * Returns the length along the y-axis of the rectangle
     *
     * @return The length along the y-axis of the rectangle
     */
    constexpr double yLength() const;

    /**
     * Returns the area of the rectangle
     *
     * @return The area of the rectangle
     */
    constexpr double area() const;

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
     * Returns a digit corresponding to a specific corner for pos%4. (0 for <-x,-y>, 1
     * for <-x,+y>, 2 for <+x,+y> and 3 for <+x,-y>)
     *
     * @param pos The position of the corner wanted
     *
     * @return Point A digit corresponding to position
     */
    Point operator[](unsigned int pos) const;

    /**
     * Translates the rectangle
     *
     * @param offset The distance to move the rectangle
     */
    void translate(const Vector &offset);

    /**
     * Checks whether a point is within the boundries of the rectangle
     *
     * @param p The point to test
     *
     * @return bool Whether the point is inside the boundary of the rectangle
     */
    constexpr bool containsPoint(const Point &p) const;

    /**
     * Attempts to move all edges of the rectangle outwards or inwards towards
     * the centre by an "amount" while maintaining the same location for the center of the
     * rectangle. The rectangle will not shrink to anything smaller than a point
     *
     * @param amount The amount to expand or shrink the recatngle by on all sides, can be
     * positive or negative
     *
     * @return bool Whether it was possible to expand or shrink the rectangle by
     * amount requested
     */
    bool expand(double amount);

    /**
     * Gives the distance between a point and the nearest point on the rectangle's
     * boundary
     *
     * @param p The point to test
     *
     * @return The distance between the point and the rectangle's boundary
     */
    double distToBoundary(Point p);

    /**
     * Returns the corner point of the rectangle that is the furthest from the input
     * point.
     *
     * @param p The point to test
     * @return The corner point that is furthest from the test point
     */
    Point furthestCorner(Point p);

    std::vector<Point> corners();

    bool operator==(const Rectangle &p) const
    {
        return this->min_corner == p.min_corner && this->diagonal == p.diagonal;
    }

   private:
    Point min_corner;
    Point diagonal;
};

inline Rectangle::Rectangle(const Point &point1, const Point &point2)
    : min_corner(point1.x() < point2.x() ? point1.x() : point2.x(),
                 point1.y() < point2.y() ? point1.y() : point2.y()),
      diagonal(
          point1.x() < point2.x() ? point2.x() - point1.x() : point1.x() - point2.x(),
          point1.y() < point2.y() ? point2.y() - point1.y() : point1.y() - point2.y())
{
}

inline Rectangle::Rectangle(const Point &neg_x_neg_y_corner, double x_length,
                            double y_length)
    : min_corner(neg_x_neg_y_corner), diagonal(x_length, y_length)

{
}

inline constexpr double Rectangle::xLength() const
{
    return diagonal.x();
}

inline constexpr double Rectangle::yLength() const
{
    return diagonal.y();
}

inline constexpr double Rectangle::area() const
{
    return diagonal.x() * diagonal.y();
}

inline Point Rectangle::centre() const
{
    return Point(min_corner + (diagonal.toVector() / 2));
}

inline Point Rectangle::posXPosYCorner() const
{
    return min_corner + diagonal.toVector();
}

inline Point Rectangle::negXPosYCorner() const
{
    return min_corner + Vector(0, diagonal.y());
}

inline Point Rectangle::negXNegYCorner() const
{
    return min_corner;
}

inline Point Rectangle::posXNegYCorner() const
{
    return min_corner + Vector(diagonal.x(), 0);
}

inline Point Rectangle::operator[](unsigned int pos) const
{
    switch (pos)
    {
        case 0:
            return negXNegYCorner();
        case 1:
            return negXPosYCorner();
        case 2:
            return posXPosYCorner();
        case 3:
            return posXNegYCorner();
        default:
            throw std::out_of_range("Rectangle only has 4 points!!!!!!");
    }
}

inline void Rectangle::translate(const Vector &offset)
{
    min_corner += offset;
}

inline constexpr bool Rectangle::containsPoint(const Point &p) const
{
    return p.x() >= min_corner.x() && p.y() >= min_corner.y() &&
           p.x() <= min_corner.x() + diagonal.x() &&
           p.y() <= min_corner.y() + diagonal.y();
}

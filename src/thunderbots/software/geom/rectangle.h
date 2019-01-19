#pragma once

#include <cmath>

#include "geom/point.h"

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
    explicit constexpr Rectangle(const Point &point1, const Point &point2);

    /**
     * Creates a new Rectangle from a corner and a size
     *
     * @param sw_corner The south-west corner of the rectangle
     *
     * @param width The width of the rectangle
     *
     * @param height The height of the rectangle
     */
    explicit constexpr Rectangle(const Point &sw_corner, double width, double height);

    /**
     * Returns the width of the rectangle
     *
     * @return The width of the rectangle
     */
    constexpr double width() const;

    /**
     * Returns the height of the rectangle
     *
     * @return The height of the rectangle
     */
    constexpr double height() const;

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
    constexpr Point centre() const;

    /**
     * Returns the north-east corner of the rectangle
     *
     * @return The north-east corner of the rectangle
     */
    constexpr Point neCorner() const;

    /**
     * Returns the north-west corner of the rectangle
     *
     * @return The north-west corner of the rectangle
     */
    constexpr Point nwCorner() const;

    /**
     * Returns the south-west corner of the rectangle
     *
     * @return The south-west corner of the rectangle
     */
    constexpr Point swCorner() const;

    /**
     * Returns the south-east corner of the rectangle
     *
     * @return The south-east corner of the rectangle
     */
    constexpr Point seCorner() const;

    /**
     * Returns a digit corresponding to a specific corner for pos%4. (0 for south-west, 1
     * for north-west, 2 for north-east and 3 for south-east)
     *
     * @param pos The position of the corner wanted
     *
     * @return Point A digit corresponding to position
     */
    constexpr Point operator[](unsigned int pos) const;

    /**
     * Translates the rectangle
     *
     * @param offset The distance to move the rectangle
     */
    void translate(const Point &offset);

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

    constexpr bool operator==(const Rectangle &p) const
    {
        return this->min_corner == p.min_corner && this->diagonal == p.diagonal;
    }

   private:
    Point min_corner;
    Point diagonal;
};

inline constexpr Rectangle::Rectangle(const Point &point1, const Point &point2)
    : min_corner(point1.x() < point2.x() ? point1.x() : point2.x(),
                 point1.y() < point2.y() ? point1.y() : point2.y()),
      diagonal(
          point1.x() < point2.x() ? point2.x() - point1.x() : point1.x() - point2.x(),
          point1.y() < point2.y() ? point2.y() - point1.y() : point1.y() - point2.y())
{
}

inline constexpr Rectangle::Rectangle(const Point &sw_corner, double width, double height)
    : min_corner(sw_corner), diagonal(width, height)
{
}

inline constexpr double Rectangle::width() const
{
    return diagonal.x();
}

inline constexpr double Rectangle::height() const
{
    return diagonal.y();
}

inline constexpr double Rectangle::area() const
{
    return diagonal.x() * diagonal.y();
}

inline constexpr Point Rectangle::centre() const
{
    return min_corner + diagonal / 2;
}

inline constexpr Point Rectangle::neCorner() const
{
    return min_corner + diagonal;
}

inline constexpr Point Rectangle::nwCorner() const
{
    return min_corner + Point(0, diagonal.y());
}

inline constexpr Point Rectangle::swCorner() const
{
    return min_corner;
}

inline constexpr Point Rectangle::seCorner() const
{
    return min_corner + Point(diagonal.x(), 0);
}

inline constexpr Point Rectangle::operator[](unsigned int pos) const
{
    switch (pos)
    {
        case 0:
            return swCorner();
        case 1:
            return nwCorner();
        case 2:
            return neCorner();
        case 3:
            return seCorner();
        default:
            throw std::out_of_range("Rectangle only has 4 points!!!!!!");
    }
}

inline void Rectangle::translate(const Point &offset)
{
    min_corner += offset;
}

inline constexpr bool Rectangle::containsPoint(const Point &p) const
{
    return p.x() >= min_corner.x() && p.y() >= min_corner.y() &&
           p.x() <= min_corner.x() + diagonal.x() &&
           p.y() <= min_corner.y() + diagonal.y();
}

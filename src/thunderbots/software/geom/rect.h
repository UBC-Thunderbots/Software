#pragma once

#include <algorithm>
#include <cmath>
#include "geom/point.h"

/**
 * A rectangle.
 */
class Rect final
{
   public:
    /**
     * Creates a new Rect from two corners.
     *
     * \param[in] point1 one of the rectangle's corners.
     *
     * \param[in] point2 the corner diagonally-opposite to \p point1.
     */
    explicit constexpr Rect(const Point &point1, const Point &point2);

    /**
     * Creates a new Rect from a corner and a size.
     *
     * \param[in] sw_corner the south-west corner of the rectangle.
     *
     * \param[in] width the width of the rectangle.
     *
     * \param[in] height the height of the rectangle.
     */
    explicit constexpr Rect(const Point &sw_corner, double width, double height);

    /**
     * Returns the width of the rectangle.
     *
     * \return the width of the rectangle.
     */
    constexpr double width() const;

    /**
     * Returns the height of the rectangle.
     *
     * \return the height of the rectangle.
     */
    constexpr double height() const;

    /**
     * Returns the area of the rectangle.
     *
     * \return the area of the rectangle.
     */
    constexpr double area() const;

    /**
     * Returns the centre of the rectangle.
     *
     * \return the centre of the rectangle.
     */
    constexpr Point centre() const;

    /**
     * Returns the north-east corner of the rectangle.
     *
     * \return the north-east corner of the rectangle.
     */
    constexpr Point neCorner() const;

    /**
     * Returns the north-west corner of the rectangle.
     *
     * \return the north-west corner of the rectangle.
     */
    constexpr Point nwCorner() const;

    /**
     * Returns the south-west corner of the rectangle.
     *
     * \return the south-west corner of the rectangle.
     */
    constexpr Point swCorner() const;

    /**
     * Returns the south-east corner of the rectangle.
     *
     * \return the south-east corner of the rectangle.
     */
    constexpr Point seCorner() const;

    /**
     * Returns 0-sw 1-nw 2-ne 3-se corner for pos%4
     *
     * \param[in] pos of corner wanted
     *
     * \return Point corresponding to position
     */
    constexpr Point operator[](unsigned int pos) const;

    /**
     * Translates the rectangle.
     *
     * \param[in] offset the distance to move the rectangle.
     */
    void translate(const Point &offset);

    /**
     * Checks whether a point is within the boundries of the rectangle
     *
     * \param[in] p the point to test
     *
     * \return bool whether the point is inside the boundry of the rectangle
     */
    constexpr bool containsPoint(Point p) const;

    /**
     * Tries to move all of the edges of the rectangle outwards/inwards towards
     * the centre by "amount"
     * while keeping the location of the centre of the rectangle the same the
     * rectangle
     * will not shrink to something smaller than a point
     *
     * \param[in] amount the amount to shrink the recatngle by on all sides
     * (positive or negative numbers ok)
     *
     * \return bool whether it was possible to expand/shrink the rectangle by
     * amount requested
     */
    bool expand(double amount);

    /**
     * Gives the distance between a point and the nearest point on the rectangle
     * boundry
     *
     * \param[in] p the point to test
     *
     * \return double
     */
    double distToBoundary(Point p);

   private:
    Point min_corner;
    Point diag;
};

inline constexpr Rect::Rect(const Point &point1, const Point &point2)
    : min_corner(
          point1.x() < point2.x() ? point1.x() : point2.x(),
          point1.y() < point2.y() ? point1.y() : point2.y()),
      diag(
          point1.x() < point2.x() ? point2.x() - point1.x() : point1.x() - point2.x(),
          point1.y() < point2.y() ? point2.y() - point1.y() : point1.y() - point2.y())
{
}

inline constexpr Rect::Rect(const Point &sw_corner, double width, double height)
    : min_corner(sw_corner), diag(width, height)
{
}

inline constexpr double Rect::width() const
{
    return diag.x();
}

inline constexpr double Rect::height() const
{
    return diag.y();
}

inline constexpr double Rect::area() const
{
    return diag.x() * diag.y();
}

inline constexpr Point Rect::centre() const
{
    return min_corner + diag / 2;
}

inline constexpr Point Rect::neCorner() const
{
    return min_corner + diag;
}

inline constexpr Point Rect::nwCorner() const
{
    return min_corner + Point(0, diag.y());
}

inline constexpr Point Rect::swCorner() const
{
    return min_corner;
}

inline constexpr Point Rect::seCorner() const
{
    return min_corner + Point(diag.x(), 0);
}

inline constexpr Point Rect::operator[](unsigned int pos) const
{
    return (pos % 4) == 0
               ? swCorner()
               : (pos % 4) == 1 ? nwCorner() : (pos % 4) == 2 ? neCorner() : seCorner();
}

inline void Rect::translate(const Point &offset)
{
    min_corner += offset;
}

inline constexpr bool Rect::containsPoint(Point p) const
{
    return p.x() >= min_corner.x() && p.y() >= min_corner.y() &&
           p.x() <= min_corner.x() + diag.x() && p.y() <= min_corner.y() + diag.y();
}

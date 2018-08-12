#pragma once

#include <functional>
#include "geom/point.h"
#include "geom/rect.h"

class Line final
{
   public:
    Point first;
    Point second;

    /**
     * \brief Creates a degenerate Line at (0, 0)
     */
    inline explicit constexpr Line()
    {
    }

    /**
     * \brief Creates a Line that starts and ends at the given points
     */
    inline explicit Line(const Point& first, const Point& second)
        : first(first), second(second)
    {
    }

    inline double slope() const
    {
        return (second.y() - first.y()) / (second.x() - first.x());
    }
};

class Seg final
{
   public:
    Point start;
    Point end;

    /**
     * \brief Creates a degenerate Seg at (0, 0)
     */
    inline explicit constexpr Seg()
    {
    }

    /**
     * \brief Creates a Seg that starts and ends at the given points
     */
    inline explicit Seg(const Point& start, const Point& end) : start(start), end(end)
    {
    }

    /**
     * \brief Creates a Seg that is this reversed.
     */
    inline Seg reverse() const
    {
        return Seg(end, start);
    }

    /**
     * \brief Makes a Point out of this Seg.
     */
    inline Point to_vector() const
    {
        return end - start;
    }

    /**
     * \brief Makes a line out of this Seg.
     */
    inline Line to_line()
    {
        return Line(start, end);
    }

    inline double slope() const
    {
        return (end.y() - start.y()) / (end.x() - start.x());
    }
};

class Ray final
{
   public:
    Point start;
    Vector dir;

    /**
     * \brief Creates a degenerate Seg at (0, 0)
     */
    inline explicit constexpr Ray()
    {
    }

    /**
     * \brief Creates a Seg that starts and contains a point along the given
     * line
     */
    inline explicit Ray(const Point& start, const Vector& dir) : start(start), dir(dir)
    {
    }

    inline Seg to_seg() const
    {
        return Seg(start, dir);
    }

    inline Point to_vector() const
    {
        return dir - start;
    }

    inline Line to_line() const
    {
        return Line(start, dir);
    }
};

class Circle final
{
   public:
    Point origin;
    double radius;

    /**
     * \brief Creates a circle with origin (0, 0) and radius 0.
     */
    inline explicit constexpr Circle() : radius(0)
    {
    }

    inline explicit constexpr Circle(const Point& origin, double r)
        : origin(origin), radius(r)
    {
    }

    // TODO: these shapes could use more operators and functions (eg area), and hashes

    inline constexpr bool operator==(const Circle& p)
    {
        return this->origin == p.origin && this->radius == p.radius;
    }

    inline constexpr bool operator!=(const Circle& p)
    {
        return !(*this == p);
    }
};

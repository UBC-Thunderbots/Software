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
     * Creates a degenerate Line at (0, 0)
     */
    inline explicit constexpr Line()
    {
    }

    /**
     * Creates a Line that starts and ends at the given points
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
     * Creates a degenerate Seg at (0, 0)
     */
    inline explicit constexpr Seg()
    {
    }

    /**
     * Creates a Seg that starts and ends at the given points
     */
    inline explicit Seg(const Point& start, const Point& end) : start(start), end(end)
    {
    }

    /**
     * Creates a Seg that is this reversed.
     */
    inline Seg reverse() const
    {
        return Seg(end, start);
    }

    /**
     * Makes a Point out of this Seg.
     */
    inline Vector toVector() const
    {
        return end - start;
    }

    /**
     * Makes a line out of this Seg.
     */
    inline Line toLine()
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
     * Creates a degenerate Seg at (0, 0)
     */
    inline explicit constexpr Ray()
    {
    }

    /**
     * Creates a Seg that starts and contains a point along the given
     * line
     */
    inline explicit Ray(const Point& start, const Vector& dir) : start(start), dir(dir)
    {
    }

    inline Seg toSeg() const
    {
        return Seg(start, dir);
    }

    inline Vector toVector() const
    {
        return dir - start;
    }

    inline Line toLine() const
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
     * Creates a circle with origin (0, 0) and radius 0.
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

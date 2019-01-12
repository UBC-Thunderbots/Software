#pragma once

#include <functional>

#include "geom/line.h"
#include "geom/point.h"
#include "geom/segment.h"

class Ray final
{
   public:
    Point start;
    Vector direction;

    /**
     * Creates a degenerate Ray at (0, 0)
     */
    inline explicit constexpr Ray() {}

    /**
     * Creates a Ray that starts and contains a point along the given
     * line
     */
    inline explicit Ray(const Point& start, const Vector& direction)
        : start(start), direction(direction)
    {
    }

    inline Segment toSegment() const
    {
        return Segment(start, direction);
    }

    inline Vector toVector() const
    {
        return direction - start;
    }

    inline Line toLine() const
    {
        return Line(start, direction);
    }
};

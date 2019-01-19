#pragma once

#include "geom/point.h"

class Line final
{
   public:
    Point first;
    Point second;

    /**
     * Creates a degenerate Line at (0, 0)
     */
    inline explicit constexpr Line() {}

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

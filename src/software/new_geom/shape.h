#pragma once

#include "software/new_geom/point.h"

/**
 * A geometric figure that encloses an area in 2D space.
 */
class Shape
{
   public:
    virtual ~Shape() = default;

    /**
     * Determines whether the given Point is contained within this Shape.
     *
     * @return whether the Point p is contained within this Shape.
     */
    virtual bool contains(const Point &p) const = 0;
};

#pragma once

#include "software/geom/shape.h"

/**
 * A shape that is convex (curved outwards).
 */
class ConvexShape : public virtual Shape
{
    /**
     * Returns the area of this Shape.
     *
     * @return The area of this Shape.
     */
    virtual double area() const = 0;
};

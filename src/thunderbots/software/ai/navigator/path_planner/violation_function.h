#pragma once

#include "geom/point.h"

class ViolationFunction
{
    /**
     * Returns the distance that a point is violating a boundary
     * by, in metres.
     * @param point a Point
     * @return the distance that a point is violating a boundary
     * by, in metres.
     */
    virtual double getViolationForPoint(const Point& point) = 0;
    virtual ~ViolationFunction()                            = default;
};
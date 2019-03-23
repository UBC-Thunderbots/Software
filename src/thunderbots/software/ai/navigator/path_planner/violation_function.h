#pragma once

#include "geom/point.h"

/**
 * The ViolationFunction is an object that represents a function
 * that can be called to evaluate the boundary violation of a
 * point on the field, or implement other soft avoidance areas
 * on the field.
 */

class ViolationFunction
{
   public:
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
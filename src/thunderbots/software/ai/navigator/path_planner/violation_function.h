#pragma once

#include "geom/point.h"

class ViolationFunction {
    virtual double getViolationForPoint(const Point& point) = 0;
    virtual ~ViolationFunction() = default;
};
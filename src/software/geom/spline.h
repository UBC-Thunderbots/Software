#pragma once

#include <vector>

#include "software/geom/point.h"

/**
 * A Spline is time parameterized piecewise function
 * composed of Polynomials over time ranges
 */
class Spline
{
   public:
    Spline() = delete;
    /**
     * Construct a spline by drawing line segments between consecutive
     * Points
     *
     * @param points Points on the spline
     */
    explicit Spline(const std::vector<Point>& points);

    /**
     * Construct a spline by drawing line segments between consecutive
     * Points
     *
     * @param points Points on the spline
     */
    Spline(const std::initializer_list<Point>& _points);
};

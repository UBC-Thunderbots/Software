#pragma once

#include <vector>

#include "software/geom/point.h"
#include "software/geom/polynomial.h"

/**
 * A Spline maps a double to a Point with a parameterized piecewise function
 * composed of Polynomials over ranges
 */
class Spline
{
   public:
    Spline() = delete;
    /**
     * Construct a spline by drawing line segments between consecutive
     * Points
     *
     * @throws std::runtime_error if points.size() == 1
     *
     * @param points Points on the spline
     */
    explicit Spline(const std::vector<Point>& points);

    /**
     * Construct a spline by drawing line segments between consecutive
     * Points
     *
     * @throws std::runtime_error if points.size() == 1
     *
     * @param points Points on the spline
     */
    Spline(const std::initializer_list<Point>& _points);

    /**
     * Calculates the value of spline evaluated at value val
     *
     * @param val value to evaluate spline
     *
     * @return value of spline evaluated at value val
     *      * if not defined by a spline then return closest start/end point
     */
    Point calculateValue(double val) const;

    /**
     * Gets size of the domain of the spline
     *
     * @return size of the domain of the spline
     */
    size_t size(void) const;

   private:
    class SplineSegment
    {
       public:
        SplineSegment(Polynomial x, Polynomial y) : x(x), y(y) {}
        const Polynomial x;
        const Polynomial y;
    };

    // segments represent the polynomials that interpolate between
    // time at index to index + 1
    std::vector<SplineSegment> segments;

    Point start_point, end_point;

    /**
     * Initialize segments with points.size() - 1 linear segments interpolating
     * the points
     * Initialize start and end points
     *
     * @throws std::runtime_error if points.size() == 1
     *
     * @param points points to interpolate
     */
    void initLinearSegments(const std::vector<Point>& points);
};

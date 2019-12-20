#pragma once

#include <vector>

#include "software/geom/polynomial.h"
#include "software/new_geom/point.h"

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
     * @throws std::invalid_argument if points.size() == 0
     *
     * @param points Points on the spline
     */
    explicit Spline(const std::vector<Point>& points);

    /**
     * Construct a spline by drawing line segments between consecutive
     * Points
     *
     * @throws std::invalid_argument if points.size() == 0
     *
     * @param points Points on the spline
     */
    Spline(const std::initializer_list<Point>& points);

    /**
     * Calculates the value of spline evaluated at value val
     *
     * @param val value to evaluate spline
     *
     * @throws std::invalid_argument if val is outside of domain
     *
     * @return value of spline evaluated at value val
     *      * if not defined by a spline then return closest start/end point
     */
    Point valueAt(double val) const;

    /**
     * Gets the number of knots in the spline including start and end points
     *
     * @return size of the spline
     */
    size_t size(void) const;

    /**
     * Gets knots in the spline including start and end points
     *
     * @return knots in the spline
     */
    const std::vector<Point> getKnots(void) const;

    /**
     * Gets start point of spline
     *
     * @return start point of spline
     */
    const Point startPoint(void) const;

    /**
     * Gets end point of spline
     *
     * @return end point of spline
     */
    const Point endPoint(void) const;

   private:
    class SplineSegment
    {
       public:
        SplineSegment(Polynomial x, Polynomial y, double start, double end)
            : x(x), y(y), start(start), end(end)
        {
        }
        const Polynomial x;
        const Polynomial y;
        const double start;
        const double end;
    };

    // segments represent the polynomials that interpolate between
    // time at index to index + 1
    std::vector<SplineSegment> segments;

    // points that connect segments
    const std::vector<Point> knots;

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

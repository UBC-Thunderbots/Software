#pragma once

#include <vector>

#include "software/geom/point.h"
#include "software/geom/polynomial2d.h"
#include "software/geom/spline2d.h"

/**
 * A linear spline is a spline composed of a series of linear 2d polynomials, ie. a series
 * of connected line segments
 */
class LinearSpline2d : Spline2d
{
   public:
    LinearSpline2d() = delete;
    /**
     * Construct a spline by drawing line segments between consecutive
     * Points
     *
     * @throws std::invalid_argument if points.size() == 0
     *
     * @param points Points on the spline
     */
    explicit LinearSpline2d(const std::vector<Point>& points);

    /**
     * Construct a spline by drawing line segments between consecutive
     * Points
     *
     * @throws std::invalid_argument if points.size() == 0
     *
     * @param points Points on the spline
     */
    LinearSpline2d(const std::initializer_list<Point>& points);

    const Point getValueAt(double val) const override;

    size_t getNumKnots(void) const override;

    std::vector<double> getKnotParametrizationValues() const override;

    const std::vector<Point> getKnots(void) const override;

    const Point getStartPoint(void) const override;

    const Point getEndPoint(void) const override;

    const std::vector<SplineSegment2d> getSplineSegments() const override;

   private:
    // The segments making up this spline, from the start to the end
    std::vector<SplineSegment2d> segments;

    // points that connect segments
    const std::vector<Point> knots;

    /**
     * Initialize segments with points.size() - 1 linear segments, interpolating
     * the given points
     *
     * @throws std::invalid_argument if points.size() <= 1
     *
     * @param points points to interpolate
     */
    void initLinearSegments(const std::vector<Point>& points);
};

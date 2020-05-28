#pragma once

#include <vector>

#include "software/new_geom/polynomial2d.h"
#include "software/new_geom/point.h"
#include "software/new_geom/spline2d.h"

/**
 * A linear spline is a spline composed of a series of linear 2d polynomials, ie. a series
 * of connected line segments
 */
class LinearSpline : Spline2d
{
   public:
    LinearSpline() = delete;
    /**
     * Construct a spline by drawing line segments between consecutive
     * Points
     *
     * @throws std::invalid_argument if points.size() == 0
     *
     * @param points Points on the spline
     */
    explicit LinearSpline(const std::vector<Point>& points);

    /**
     * Construct a spline by drawing line segments between consecutive
     * Points
     *
     * @throws std::invalid_argument if points.size() == 0
     *
     * @param points Points on the spline
     */
    LinearSpline(const std::initializer_list<Point>& points);

    /**
     * Please see parent class for jdoc
     */
    const Point getValueAt(double val) const override;

    /**
     * Please see parent class for jdoc
     */
    size_t getNumKnots(void) const override;

    /**
     * Please see parent class for jdoc
     */
    const std::vector<Point> getKnots(void) const override;

    /**
     * Please see parent class for jdoc
     */
    const Point getStartPoint(void) const override;

    /**
     * Please see parent class for jdoc
     */
    const Point getEndPoint(void) const override;

    /**
     * Please see parent class for jdoc
     */
    const std::vector<SplineSegment2d> getSplineSegments() const override;

   private:
    // segments represent the polynomials that interpolate between
    // time at index to index + 1
    std::vector<SplineSegment2d> segments;

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

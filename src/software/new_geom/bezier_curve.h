#pragma once

#include "software/new_geom/point.h"
#include "software/new_geom/spline2d.h"
#include <vector>

// TODO: should this extend anything else in the geometry hierarchy?
// TODO: jdoc
// TODO: should link to https://www.ibiblio.org/e-notes/Splines/bezier.html and wikipedia
class BezierCurve2d : public SplineSegment2d {
   public:
    BezierCurve2d() = delete;

    /**
     * Construct a bezier curve from the given control points
     * @param control_points
     */
    BezierCurve2d(std::vector<Point> control_points);

    /**
     * Calculates the value of the curve evaluated at value val
     *
     * @param val A value in [0,1] to get the value of the curve at.
     *            0 is one endpoint of the curve
     *            1 is the other endpoint of the curve
     *
     * @return value of curve evaluated at value val. If val not in [0,1]
     *         then the closest endpoint
     */
    // TODO: `val` is a terrible argument name since we're getting the `value`. Rename.
    const Point getValueAt(double val) const;

    double getStartVal() const;

    double getEndVal() const;

    Polynomial2d getPolynomial() const;

   private:
    std::vector<Point> control_points;
};
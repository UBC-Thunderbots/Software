#pragma once

#include <vector>

#include "software/new_geom/point.h"
#include "software/new_geom/spline2d.h"

// TODO: should this extend anything else in the geometry hierarchy?
// TODO: jdoc
// TODO: should link to https://www.ibiblio.org/e-notes/Splines/bezier.html and wikipedia
class BezierCurve2d
{
   public:
    BezierCurve2d() = delete;

    /**
     * Construct a bezier curve from the given control points
     * @param control_points The control points for the bezier curve
     *
     * @throws std::invalid_argument If there are not at least two control points
     */
    explicit BezierCurve2d(std::vector<Point> control_points);

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

    // TODO: jdoc, make sure to note that parametrization is in [0,1]
    Polynomial2d getPolynomial() const;

   private:
    /*
     * Evaluate the point `t` on the bezier curve with control points `points`
     *
     * Implementation of the De Casteljau algorithm:
     * https://en.wikipedia.org/wiki/De_Casteljau%27s_algorithm
     * Assumes the curve is parameterized such that `t=0` is the first point, and
     * `t=1` is the last point.
     *
     * @param points The control points of the bezier curve
     * @param t The value at which to evaluate the bezier curve
     *
     * @throw std::invalid_arguments If `points` is empty
     *
     * @return The point on the bezier curve corresponding to the value `t`
     */
    static const Point deCasteljauAlgorithm(const std::vector<Point>& points,
                                            const double t);

    /**
     * Compute the x and y coefficients for the given order on the polynomial
     * representation of this curve
     *
     * @param order The order to get the polynomial coefficients for
     *
     * @return The x and y polynomial coefficients
     */
    const Vector computePolynomialCoefficients(const size_t order) const;

    // The control points for this bezier curve
    const std::vector<Point> control_points;
};
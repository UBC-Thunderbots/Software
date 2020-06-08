#pragma once

#include <vector>

#include "software/new_geom/point.h"
#include "software/new_geom/spline2d.h"

/**
 * A 2D bezier curve, parametrized on [0,1]
 *
 * More info:
 * https://www.ibiblio.org/e-notes/Splines/bezier.html
 * https://en.wikipedia.org/wiki/B%C3%A9zier_curve
 */
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
     * Calculates the value of the curve evaluated at value t
     *
     * @param t A value in [0,1] to get the value of the curve at.
     *          0 is one endpoint of the curve
     *          1 is the other endpoint of the curve
     *
     * @return value of curve evaluated at t. If t not in [0,1]
     *         then the closest endpoint.
     */
    const Point getValueAt(double t) const;

    /**
     * Get the polynomial representation of this curve
     *
     * @return The polynomial representation of this curve, parametrized such that the
     *         value at t=0 is the start of the curve, and the value at t=1 is the
     *         end of the curve
     */
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

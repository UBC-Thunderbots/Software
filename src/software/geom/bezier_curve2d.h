#pragma once

#include <vector>

#include "software/geom/point.h"
#include "software/geom/spline2d.h"

/**
 * A 2D bezier curve, parametrized on [0,1]
 *
 * A bezier curve is a curve defined by a series of control points, that interpolates
 * the start end point, with the middle points "dragging" the curve to change it's shape.
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
     *
     * @param control_points The control points for the bezier curve
     *
     * @throws std::invalid_argument If there are not at least two control points
     */
    explicit BezierCurve2d(std::vector<Point> control_points);

    /**
     * Calculates the value of the curve evaluated at value t
     *
     * @param t A value in [0,1] to get the value of the curve at.
     *          0 is first control point
     *          1 is last control point
     *
     * @return value of curve evaluated at t. If t is not in [0,1]
     *         then the closest endpoint.
     */
    const Point getValueAt(double t) const;

    /**
     * Get the polynomial representation of this curve
     *
     * @return The polynomial representation of this curve, parametrized such that the
     *         value at t=0 is the first control point, and the value at t=1 is the
     *         last control point
     */
    Polynomial2d getPolynomial() const;

    /**
     * The equality operator for BezierCurve2d
     * @param other The other BezierCurve2d to compare this one to for equality.
     * @return True if this curve is equal to other
     */
    bool operator==(const BezierCurve2d& other) const;

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
     * Compute the x and y coefficients for the given order_of_coefficients on the
     * polynomial representation of this curve
     *
     * @param order_of_coefficients The order to get the polynomial coefficients for
     *
     * @return The x and y polynomial coefficients
     */
    const Vector computePolynomialCoefficients(const size_t order_of_coefficients) const;

    // The control points for this bezier curve
    const std::vector<Point> control_points;
};

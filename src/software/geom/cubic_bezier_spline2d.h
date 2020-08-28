#pragma once

#include "software/geom/bezier_curve2d.h"
#include "software/geom/spline2d.h"

/**
 * A Cubic Bezier spline is a C2 continuous spline that is formed by a series of bezier
 * curves.
 *
 * Some useful resources:
 * https://www.ibiblio.org/e-notes/Splines/Intro.htm
 * https://www.ibiblio.org/e-notes/Splines/b-int.html
 */
class CubicBezierSpline2d : public Spline2d
{
   public:
    CubicBezierSpline2d() = delete;

    /**
     * Create a CubicBezierSpline2d that:
     *  - starts from `start` and is tangent to `start_vector` at the start
     *  - ends at `end` and is tangent to `end_vector` at the end
     *
     *    ** EXAMPLE: **
     *
     *                           XXXXXXXXX
     *        start_vector  XXXXX         XXXXX   end_vector
     *              ^     XXX                 XXX     ^
     *              |    XX                     XX    |
     *              |   XX                       XX   |
     *              |  XX                         XX  |
     *              |  X                           X  |
     *              | XX                           XX |
     *              | X                             X |
     *              | X                             X |
     *              |XX                             XX|
     *              |X                               X|
     *              |X                               X|
     *              |X                               X|
     *       start  +                                 +  end
     *
     *
     * @param start_point The first point on the spline
     * @param start_vector The tangent vector at the start of the spline
     * @param end_point The last point on the spline
     * @param end_vector The tangent vector at the end of the spline.
     *                   NOTE: this points back along the body of the spline, instead
     *                         of "off the end" of the spline. ie. this vector is
     *                         "symmetric" with `start_vector`
     * @param intermediate_knots The intermediate points between `start_point` and
     *                                    `end_point` that this spline will pass through
     *
     * The direction of the start and end vectors determine the direction of the curve at
     * the start and end points. The magnitude of these vectors determine how far the
     * curve is stretched in the direction of the vector before connecting with the
     * intermediate points
     */
    explicit CubicBezierSpline2d(const Point& start_point, const Vector& start_vector,
                                 const Point& end_point, const Vector& end_vector,
                                 const std::vector<Point>& intermediate_knots);

    const Point getValueAt(double t) const override;
    const std::vector<Point> getKnots() const override;
    size_t getNumKnots() const override;
    std::vector<double> getKnotParametrizationValues() const override;
    const Point getStartPoint() const override;
    const Point getEndPoint() const override;
    const std::vector<SplineSegment2d> getSplineSegments() const override;

    /**
     * Get the control points that are the implicit representation of the bezier
     * curves that form this spline.
     *
     * @return The control points that represent the the bezier curves that form this
     *         spline.
     */
    const std::vector<Point>& getControlPoints() const;

    /**
     * Get the number of segments on this spline.
     *
     * @return The number of segments on this spline.
     */
    size_t getNumSegments() const;


   private:
    // The degree of the bezier curves that make up this spline
    static const size_t BEZIER_CURVE_DEGREE = 3;

    /**
     * Get the segment of this spline at the given index
     *
     * @param index The index of the segment on this spline. Must be in
     *              [0, getNumSegments()-1]
     *
     * @return The segment of this spline at the given index
     */
    BezierCurve2d getSegmentAtIndex(size_t index) const;

    /**
     * Compute the control points that represent the bezier curves that satisfy the
     * constraints of this spline
     *
     * @param start_point The first point on the spline
     * @param start_vector The tangent vector at the start of the spline
     * @param end_point The last point on the spline
     * @param end_vector The tangent vector at the end of the spline.
     *                   NOTE: this points back along the body of the spline, instead
     *                         of "off the end" of the spline. ie. this vector is
     *                         "symmetric" with `start_vector`
     * @param intermediate_knots The intermediate points between `start_point` and
     *                                    `end_point` that this spline will pass through
     * @return A vector of control points that are the implicit representation of the
     *         bezier curves that make up the spline represented by the given args. ie.
     *         0th-3rd (inclusive) points are the control points of the first bezier
     *         curve, 3rd-6th (inclusive) points are the control points of the second
     *         bezier curve, etc.
     */
    std::vector<Point> computeControlPoints(const Point& start_point,
                                            const Vector& start_vector,
                                            const Point& end_point,
                                            const Vector& end_vector,
                                            const std::vector<Point>& intermediate_knots);

    // The control points that are the implicit representation of the bezier curves that
    // form this splines.
    // 0th-3rd (inclusive) points are the control points of the first bezier curve
    // 3rd-6th (inclusive) points are the control points of the second bezier curve
    // etc.
    std::vector<Point> control_points;
};

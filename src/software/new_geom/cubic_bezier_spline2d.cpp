#include "software/new_geom/cubic_bezier_spline2d.h"

#include <assert.h>

#include <algorithm>

#include "software/new_geom/bezier_curve2d.h"

CubicBezierSpline2d::CubicBezierSpline2d(const Point& start_point,
                                         const Vector& start_vector,
                                         const Point& end_point, const Vector& end_vector,
                                         std::vector<Point> intermediate_knots)
    : control_points(computeControlPoints(start_point, start_vector, end_point,
                                          end_vector, intermediate_knots))
{
}

const Point CubicBezierSpline2d::getValueAt(double t) const
{
    // Find which segment of this spline `t` falls on, assuming the start of
    // the first segment is t=0, the end of the last segment is t=1, and the
    // the knots are evenly distributed in between 0 and 1

    // TODO: consider edge case where t is _exactly_ 1

    // The below algorithm does not handle the case where t is exactly 1, so we
    // check explicitly for that here
    if (t == 1)
    {
        return control_points.back();
    }

    const double t_constrained = std::clamp(t, 0.0, 1.0);

    // TODO: comment here
    const double t_mapped = t_constrained * static_cast<double>(getNumSegments());

    // TODO: comment here
    const int segment_index = std::floor(t_mapped);

    // TODO: comment here
    const double t_on_segment = (t_mapped - static_cast<double>(segment_index));

    // TODO: comment here
    // TODO: better name for this var
    const size_t i = static_cast<size_t>(segment_index) * 3;
    return BezierCurve2d({control_points[i], control_points[i + 1], control_points[i + 2],
                          control_points[i + 3]})
        .getValueAt(t_on_segment);
}

const std::vector<Point> CubicBezierSpline2d::getKnots() const
{
    std::vector<Point> knots;
    for (size_t i = 0; i < control_points.size(); i += 3)
    {
        knots.emplace_back(control_points[i]);
    }

    return knots;
}

const std::vector<Point>& CubicBezierSpline2d::getControlPoints() const
{
    return control_points;
}

size_t CubicBezierSpline2d::getNumKnots() const
{
    return static_cast<size_t>((control_points.size() - 1) / 3 + 1);
}

size_t CubicBezierSpline2d::getNumSegments() const
{
    return getNumKnots() - 1;
}


std::vector<double> CubicBezierSpline2d::getKnotVector() const
{
    // We assume a linear spacing of all the knots from 0 to 1
    std::vector<double> knot_vector;
    const double num_knots = getNumKnots();
    for (size_t i = 0; i < num_knots; i++)
    {
        knot_vector.emplace_back(i * 1.0 / static_cast<double>(num_knots-1));
    }
    return knot_vector;
}

const Point CubicBezierSpline2d::getStartPoint() const
{
    // We are guaranteed to at least have two control points, the start and end points
    return control_points.front();
}

const Point CubicBezierSpline2d::getEndPoint() const
{
    // We are guaranteed to at least have two control points, the start and end points
    return control_points.back();
}

const std::vector<SplineSegment2d> CubicBezierSpline2d::getSplineSegments() const
{
    std::vector<SplineSegment2d> segments;
    const double num_knots = getNumKnots();
    for (size_t i = 0; i < num_knots - 1; i++)
    {
        const BezierCurve2d bezier_curve(
            {control_points[i * 3], control_points[i * 3 + 1], control_points[i * 3 + 2],
             control_points[i * 3 + 3]});
        segments.emplace_back(createSplineSegment2d(0, 1, bezier_curve.getPolynomial()));
    }
    return segments;
}

std::vector<Point> CubicBezierSpline2d::computeControlPoints(
    const Point& start_point, const Vector& start_vector, const Point& end_point,
    const Vector& end_vector, std::vector<Point> intermediate_knots)
{
    // TODO: cleanup, better naming, better comments
    // Link to https://www.ibiblio.org/e-notes/Splines/b-int.html

    std::vector<Point> all_knots;
    all_knots.emplace_back(start_point);
    all_knots.insert(all_knots.end(), intermediate_knots.begin(),
                     intermediate_knots.end());
    all_knots.emplace_back(end_point);

    const size_t n = all_knots.size();

    std::vector<Vector> A(n - 2, Vector(0, 0));
    std::vector<double> B(n - 2, 0);

    if (n - 2 > 0)
    {
        B[0] = -0.25;
        A[0] = (all_knots[2] - all_knots[0] - start_vector) / 4;
    }
    for (size_t i = 1; i < A.size(); i++)
    {
        A[i] = (all_knots[i + 2] - all_knots[i] - A[i - 1]) / (4 + B[i - 1]);
        B[i] = -1.0 / (4.0 + B[i - 1]);
    }

    std::vector<Vector> d(n, Vector(0, 0));
    d[0]     = start_vector;
    d[n - 1] = -end_vector;

    for (size_t i = n - 2; i > 0; i--)
    {
        d[i] = A[i - 1] + B[i - 1] * d[i + 1];
    }

    std::vector<Point> control_points;
    for (size_t i = 0; i < all_knots.size(); i++)
    {
        if (i > 0)
        {
            control_points.emplace_back(all_knots[i] - d[i]);
        }
        control_points.emplace_back(all_knots[i]);
        if (i < n - 1)
        {
            control_points.emplace_back(all_knots[i] + d[i]);
        }
    }

    // TODO: figure out what this should be
    //    assert(control_points.size()-1 % 3 == 0);

    return control_points;
}
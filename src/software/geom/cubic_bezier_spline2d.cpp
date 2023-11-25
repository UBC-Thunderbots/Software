#include "software/geom/cubic_bezier_spline2d.h"

#include <assert.h>

#include <algorithm>

CubicBezierSpline2d::CubicBezierSpline2d(const Point& start_point,
                                         const Vector& start_vector,
                                         const Point& end_point, const Vector& end_vector,
                                         const std::vector<Point>& intermediate_knots)
    : control_points(computeControlPoints(start_point, start_vector, end_point,
                                          end_vector, intermediate_knots))
{
}

const Point CubicBezierSpline2d::getValueAt(double t) const
{
    // Find which segment of this spline `t` falls on, assuming the start of
    // the first segment is t=0, the end of the last segment is t=1, and the
    // the knots are evenly distributed in-between 0 and 1

    const double t_constrained = std::clamp(t, 0.0, 1.0);

    // Scale t from [0,1] -> [0,n] where n is the number of segments
    const double t_mapped = t_constrained * static_cast<double>(getNumSegments());

    // In most cases the segment index is just the floor of the mapped t, but we
    // also need to have extra handling for the case where t=1, in which case if we
    // took the floor we'd be accessing a segment that doesn't exist (ie. one that
    // _starts_ at the end of the spline). So for this case we just use the last segment
    const size_t segment_index =
        t < 1 ? static_cast<size_t>(std::floor(t_mapped)) : getNumSegments() - 1;

    // Figure out what the t-value on the segment is going to be
    const double t_on_segment = (t_mapped - static_cast<double>(segment_index));

    // Generate the bezier curve for the segment and evaluate it at the segment t-value
    const BezierCurve2d segment = getSegmentAtIndex(segment_index);
    return segment.getValueAt(t_on_segment);
}

const std::vector<Point> CubicBezierSpline2d::getKnots() const
{
    std::vector<Point> knots;
    for (size_t i = 0; i < control_points.size(); i += BEZIER_CURVE_DEGREE)
    {
        knots.emplace_back(control_points[i]);
    }

    return knots;
}

size_t CubicBezierSpline2d::getNumKnots() const
{
    // We are guaranteed to have BEZIER_CURVE_DEGREE*n + 1 control points (where `n` is
    // some integer > 0) so we can safely do integer division here.
    return static_cast<size_t>((control_points.size() - 1) / BEZIER_CURVE_DEGREE + 1);
}

std::vector<double> CubicBezierSpline2d::getKnotParametrizationValues() const
{
    // We assume a linear spacing of all the knots from 0 to 1
    std::vector<double> knot_vector;
    const size_t num_knots = getNumKnots();
    for (size_t i = 0; i < num_knots; i++)
    {
        knot_vector.emplace_back(static_cast<double>(i) * 1.0 /
                                 static_cast<double>(num_knots - 1));
    }
    return knot_vector;
}

const Point CubicBezierSpline2d::getStartPoint() const
{
    // We are guaranteed to at least have four control points, the start and end points,
    // along with `start + start_vector` and `end + end_vector`
    return control_points.front();
}

const Point CubicBezierSpline2d::getEndPoint() const
{
    // We are guaranteed to at least have four control points, the start and end points,
    // along with `start + start_vector` and `end + end_vector`
    return control_points.back();
}

const std::vector<SplineSegment2d> CubicBezierSpline2d::getSplineSegments() const
{
    std::vector<SplineSegment2d> segments;
    const size_t num_segments = getNumSegments();
    for (size_t i = 0; i < num_segments; i++)
    {
        const BezierCurve2d curve = getSegmentAtIndex(i);
        segments.emplace_back(createSplineSegment2d(0, 1, curve.getPolynomial()));
    }
    return segments;
}

const std::vector<Point>& CubicBezierSpline2d::getControlPoints() const
{
    return control_points;
}

size_t CubicBezierSpline2d::getNumSegments() const
{
    return getNumKnots() - 1;
}

BezierCurve2d CubicBezierSpline2d::getSegmentAtIndex(size_t index) const
{
    return BezierCurve2d({control_points[index * BEZIER_CURVE_DEGREE],
                          control_points[index * BEZIER_CURVE_DEGREE + 1],
                          control_points[index * BEZIER_CURVE_DEGREE + 2],
                          control_points[index * BEZIER_CURVE_DEGREE + 3]});
}

std::vector<Point> CubicBezierSpline2d::computeControlPoints(
    const Point& start_point, const Vector& start_vector, const Point& end_point,
    const Vector& end_vector, const std::vector<Point>& intermediate_knots)
{
    // The math here is based off of: https://www.ibiblio.org/e-notes/Splines/b-int.html
    // And the variable names have been maintained (where reasonably possible) to match
    // those found at this aforementioned link.

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

    assert((control_points.size() - 1) % BEZIER_CURVE_DEGREE == 0);

    return control_points;
}

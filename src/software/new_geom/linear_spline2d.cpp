#include "software/new_geom/linear_spline2d.h"

#include <algorithm>
#include <sstream>

LinearSpline2d::LinearSpline2d(const std::vector<Point>& points) : knots(points)
{
    initLinearSegments(points);
}

LinearSpline2d::LinearSpline2d(const std::initializer_list<Point>& points) : knots(points)
{
    initLinearSegments(points);
}

const Point LinearSpline2d::getValueAt(double val) const
{
    double clamped_val = std::clamp(val, 0.0, 1.0);

    Point retval;

    if (segments.empty())
    {
        retval = knots.front();
    }
    else
    {
        // Note: this could be more performant with binary search
        auto seg_it = std::find_if(segments.begin(), segments.end(),
                                   [clamped_val](const SplineSegment2d& sseg) {
                                       return (clamped_val >= sseg.getStartVal() &&
                                               clamped_val <= sseg.getEndVal());
                                   });

        if (seg_it == segments.end())
        {
            std::stringstream ss;
            ss << "Tried to evaluate spline at " << clamped_val
               << ", which was not in any interval of any segment";
            throw std::runtime_error(ss.str());
        }

        retval = seg_it->getPolynomial().valueAt(clamped_val);
    }

    return retval;
}

size_t LinearSpline2d::getNumKnots(void) const
{
    return knots.size();
}

const std::vector<Point> LinearSpline2d::getKnots(void) const
{
    return knots;
}

const Point LinearSpline2d::getStartPoint(void) const
{
    return knots.front();
}

const Point LinearSpline2d::getEndPoint(void) const
{
    return knots.back();
}

const std::vector<SplineSegment2d> LinearSpline2d::getSplineSegments() const
{
    return segments;
}

void LinearSpline2d::initLinearSegments(const std::vector<Point>& points)
{
    if (points.size() == 0)
    {
        throw std::invalid_argument("Cannot create spline with no points");
    }
    else if (points.size() > 1)
    {
        // only splines with more than one point can have segments
        for (size_t i = 1; i < points.size(); i++)
        {
            double input_start =
                static_cast<double>(i - 1) / static_cast<double>(points.size() - 1);
            double input_end =
                static_cast<double>(i) / static_cast<double>(points.size() - 1);

            Polynomial1d poly_x = Polynomial1d::constructLinearPolynomialFromConstraints(
                input_start, points[i - 1].x(), input_end, points[i].x());
            Polynomial1d poly_y = Polynomial1d::constructLinearPolynomialFromConstraints(
                input_start, points[i - 1].y(), input_end, points[i].y());
            Polynomial2d poly2d(poly_x, poly_y);
            segments.push_back(createSplineSegment2d(input_start, input_end, poly2d));
        }
    }
}

#include "software/geom/spline.h"

Spline::Spline(const std::vector<Point>& points)
    : knots(points), domain(std::make_pair(0.0, 1.0))
{
    initLinearSegments(points);
}

Spline::Spline(const std::initializer_list<Point>& points)
    : knots(points), domain(std::make_pair(0.0, 1.0))
{
    initLinearSegments(points);
}

Point Spline::valueAt(double val) const
{
    if (val < domain.first || val > domain.second)
    {
        throw std::invalid_argument("val is outside of the domain of this spline");
    }

    if (val == domain.first)
    {
        return knots.front();
    }

    if (val == domain.second)
    {
        return knots.back();
    }

    // assume that the segments are equally spaced in domain
    int index = (int)(val * knots.size());
    if (val < segments[index].start || val > segments[index].end)
    {
        throw std::runtime_error("Segments are not equally spaced as expected");
    }
    double x_val = segments[index].x.valueAt(val);
    double y_val = segments[index].y.valueAt(val);
    return Point(x_val, y_val);
}

size_t Spline::size(void) const
{
    return knots.size();
}

const std::vector<Point> Spline::getKnots(void) const
{
    return knots;
}

const std::pair<double, double> Spline::getDomain(void) const
{
    return domain;
}

void Spline::initLinearSegments(const std::vector<Point>& points)
{
    if (points.size() == 0)
    {
        throw std::runtime_error("Cannot create spline with no points");
    }
    else if (points.size() > 1)
    {
        // only splines with more than one point can have segments
        for (size_t i = 1; i < points.size(); i++)
        {
            double input_start = (i - 1) / ((double)points.size() - 1);
            double input_end   = (i) / ((double)points.size() - 1);

            Polynomial poly_x = Polynomial(std::make_pair(input_start, points[i - 1].x()),
                                           std::make_pair(input_end, points[i].x()));
            Polynomial poly_y = Polynomial(std::make_pair(input_start, points[i - 1].y()),
                                           std::make_pair(input_end, points[i].y()));
            segments.push_back(SplineSegment(poly_x, poly_y, input_start, input_end));
        }
    }
}

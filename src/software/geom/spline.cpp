#include "software/geom/spline.h"

Spline::Spline(const std::vector<Point>& points) : knots(points)
{
    initLinearSegments(points);
}

Spline::Spline(const std::initializer_list<Point>& points) : knots(points)
{
    initLinearSegments(points);
}

Point Spline::valueAt(double val) const
{
    if (val < 0)
    {
        return knots.front();
    }

    if (val >= segments.size())
    {
        return knots.back();
    }

    double x_val = segments[(size_t)val].x.valueAt(val);
    double y_val = segments[(size_t)val].y.valueAt(val);
    return Point(x_val, y_val);
}

size_t Spline::size(void) const
{
    return knots.size();
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
            Polynomial poly_x = Polynomial(std::make_pair(i - 1, points[i - 1].x()),
                                           std::make_pair(i, points[i].x()));
            Polynomial poly_y = Polynomial(std::make_pair(i - 1, points[i - 1].y()),
                                           std::make_pair(i, points[i].y()));
            segments.push_back(SplineSegment(poly_x, poly_y, points[i - 1], points[i]));
        }
    }
}

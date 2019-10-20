#include "software/geom/spline.h"

Spline::Spline(const std::vector<Point>& points)
{
    initLinearSegments(points);
}

Spline::Spline(const std::initializer_list<Point>& points)
{
    initLinearSegments(points);
}

Point Spline::calculateValue(double val)
{
    if (val < 0)
    {
        return start_point;
    }

    if (val >= segments.size())
    {
        return end_point;
    }

    double x_val = segments[(size_t)val].x.calculateValue(val);
    double y_val = segments[(size_t)val].y.calculateValue(val);
    return Point(x_val, y_val);
}

size_t Spline::size(void)
{
    return segments.size();
}

void Spline::initLinearSegments(const std::vector<Point>& points)
{
    if (points.size() > 0)
    {
        for (size_t i = 1; i < points.size(); i++)
        {
            Polynomial poly_x = Polynomial(std::make_pair(i - 1, points[i - 1].x()),
                                           std::make_pair(i, points[i].x()));
            Polynomial poly_y = Polynomial(std::make_pair(i - 1, points[i - 1].y()),
                                           std::make_pair(i, points[i].y()));
            segments.push_back(SplineSegment(poly_x, poly_y));
        }

        start_point = points.front();
        end_point   = points.back();
    }
}

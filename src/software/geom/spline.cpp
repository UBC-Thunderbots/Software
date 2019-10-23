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
    if (val < 0.0 || val > 1.0)
    {
        std::stringstream ss;
        ss << "Tried to evaluate spline at " << val
           << ", which is outside of domain of the spline: [0,1]";
        throw std::invalid_argument(ss.str());
    }

    if (val == 0.0)
    {
        return knots.front();
    }

    if (val == 1.0)
    {
        return knots.back();
    }

    // assume that the segments are equally spaced in [0,1]
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

const Point Spline::startPoint(void) const
{
    return knots.front();
}

const Point Spline::endPoint(void) const
{
    return knots.back();
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

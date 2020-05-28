#include "software/geom/spline.h"

#include <algorithm>
#include <sstream>

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

    Point retval;

    if (segments.empty())
    {
        retval = knots.front();
    }
    else
    {
        // Note: this could be more performant with binary search
        auto seg_it = std::find_if(segments.begin(), segments.end(),
                                   [val](const SplineSegment& sseg) {
                                       return (val >= sseg.start && val <= sseg.end);
                                   });

        if (seg_it == segments.end())
        {
            std::stringstream ss;
            ss << "Tried to evaluate spline at " << val
               << ", which was not in any interval of any segment";
            throw std::runtime_error(ss.str());
        }

        retval = Point(seg_it->x.valueAt(val), seg_it->y.valueAt(val));
    }

    return retval;
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

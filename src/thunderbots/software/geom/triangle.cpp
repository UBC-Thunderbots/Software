#include "geom/triangle.h"

#include <algorithm>
#include <unordered_set>

#include "geom/util.h"

Triangle::Triangle(Point p0, Point p1, Point p2) : Polygon(std::vector<Point>{p0, p1, p2})
{
    segments[0] = Segment{p0, p1};
    segments[1] = Segment{p1, p2};
    segments[2] = Segment{p2, p0};
}

Triangle::Triangle(const std::vector<Point>& _points) : Polygon(_points)
{
    segments[0] = Segment(_points[0], _points[1]);
    segments[1] = Segment(_points[1], _points[2]);
    segments[2] = Segment(_points[2], _points[0]);
}

Triangle::Triangle(const std::initializer_list<Point>& _points) : Polygon(_points)
{
    for (auto i = 0; i < 3; i++)
    {
        // add a segment between consecutive points, but wrap index
        // to draw a segment from the last point to first point.
        segments[i] =
            Segment{*(points.begin() + i), *(points.begin() + ((i + 1) % points.size()))};
    }

    segments[0] = Segment(*(points.begin()), *(points.begin() + 1));
    segments[1] = Segment(*(points.begin() + 1), *(points.begin() + 2));
    segments[2] = Segment(*(points.begin() + 2), *(points.begin()));
}

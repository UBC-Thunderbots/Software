#include "geom/triangle.h"

#include <algorithm>
#include <unordered_set>

#include "geom/util.h"

Triangle::Triangle(Point p0, Point p1, Point p2)
    : segments(3), points(std::vector<Point>{p0, p1, p2})
{
    segments[0] = Segment{p0, p1};
    segments[1] = Segment{p1, p2};
    segments[2] = Segment{p2, p0};
}

Triangle::Triangle(const std::vector<Point>& _points)
    : segments(_points.size()), points(_points)
{
    segments[0] = Segment(_points[0], _points[1]);
    segments[1] = Segment(_points[1], _points[2]);
    segments[2] = Segment(_points[2], _points[0]);
}

Triangle::Triangle(const std::initializer_list<Point>& _points)
    : segments(_points.size()), points(_points)
{
    for (auto i = 0; i < _points.size(); i++)
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

bool Triangle::containsPoint(const Point& point) const
{
    // cast a ray from the point in the +x direction
    Ray ray(point, point + Point(1, 0));
    unsigned int num_intersections = 0;

    std::vector vec = getPoints();
    if (std::find(vec.begin(), vec.end(), point) != vec.end())
        return false;  // return false if point is on a corner
    else
    {
        for (const Segment& seg : segments)
        {
            // one-definition rule, use anonymous namespace for
            // intersects() in geom/util.h
            if (::intersects(ray, seg))
            {
                num_intersections++;
            }
        }

        // if the ray intersects the triangle exactly once
        // it is inside the polygon, otherwise it is outside the triangle
        return num_intersections == 1;
    }
}

bool Triangle::intersects(const Segment& segment) const
{
    for (const auto& seg : segments)
    {
        if (::intersects(seg, segment))
            return true;
    }
    return false;
}

bool Triangle::intersects(const Ray& ray) const
{
    for (const auto& seg : segments)
    {
        if (::intersects(seg, ray))
            return true;
    }
    return false;
}

const std::vector<Segment>& Triangle::getSegments() const
{
    return segments;
}

const std::vector<Point>& Triangle::getPoints() const
{
    return points;
}

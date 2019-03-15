#include "geom/polygon.h"

#include <unordered_set>

#include "geom/util.h"


Polygon::Polygon(const std::vector<Point>& _points)
    : segments(_points.size()), points(_points)
{
    for (auto i = 0; i < _points.size(); i++)
    {
        // add a segment between consecutive points, but wrap index
        // to draw a segment from the last point to first point.
        segments[i] = Segment{_points[i], _points[(i + 1) % _points.size()]};
    }
}

Polygon::Polygon(const std::initializer_list<Point>& _points)
    : segments(_points.size()), points(_points)
{
    for (auto i = 0; i < _points.size(); i++)
    {
        // add a segment between consecutive points, but wrap index
        // to draw a segment from the last point to first point.
        segments[i] =
            Segment{*(points.begin() + i), *(points.begin() + ((i + 1) % points.size()))};
    }
}

bool Polygon::containsPoint(const Point& point) const
{
    // cast a ray from the point in the +x direction
    Ray ray(point, point + Point(1, 0));
    unsigned int num_intersections = 0;
    for (const Segment& seg : segments)
    {
        // one-definition rule, use anonymous namespace for
        // intersects() in geom/util.h
        if (::intersects(ray, seg))
        {
            num_intersections++;
        }
    }

    // if the ray intersects the polygon an odd number of times,
    // it is inside the polygon, otherwise it is outside the polygon
    // see
    // https://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon
    return num_intersections % 2 != 0;
}

bool Polygon::intersects(const Segment& segment) const
{
    for (const auto& seg : segments)
    {
        if (::intersects(seg, segment))
            return true;
    }
    return false;
}

bool Polygon::intersects(const Ray& ray) const
{
    for (const auto& seg : segments)
    {
        if (::intersects(seg, ray))
            return true;
    }
    return false;
}

const std::vector<Segment>& Polygon::getSegments() const
{
    return segments;
}

const std::vector<Point>& Polygon::getPoints() const
{
    return points;
}

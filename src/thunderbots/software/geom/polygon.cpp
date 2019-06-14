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
    // This algorithm is from https://stackoverflow.com/a/16391873
    // but does not include the bounding boxes.
    //
    // A quick description of the algorithm (also from the same post) is as follows:
    // "I run a semi-infinite ray horizontally (increasing x, fixed y) out from the test
    // point, and count how many edges it crosses. At each crossing, the ray switches
    // between inside and outside. This is called the Jordan curve theorem."
    //
    // NOTE: This algorithm will treat boundaries on the bottom-left of the polygon
    // different from the boundaries on the top-right of the polygon. This small
    // inconsistency does not matter for our use cases, and actually has the benefit that
    // should two distinct polygons share an edge, any point along this edge will be
    // located in one and only one polygon.
    bool point_is_contained = false;
    int i                   = 0;
    int j                   = points.size() - 1;
    while (i < points.size())
    {
        if (((points.at(i).y() > point.y()) != (points.at(j).y() > point.y())) &&
            (point.x() < (points.at(j).x() - points.at(i).x()) *
                                 (point.y() - points.at(i).y()) /
                                 (points.at(j).y() - points.at(i).y()) +
                             points.at(i).x()))
        {
            point_is_contained = !point_is_contained;
        }

        j = i++;
    }

    return point_is_contained;
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

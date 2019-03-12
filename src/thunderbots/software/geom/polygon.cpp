#include "geom/polygon.h"

#include <unordered_set>

#include "geom/util.h"

// CMake defines NDEBUG for Release build configurations
#ifndef NDEBUG
#define VALIDATE_POLYGON
#endif

Polygon::Polygon(const std::vector<Segment>& _segments) : segments(_segments)
{
#ifdef VALIDATE_POLYGON
    if (!isValid())
    {
        throw std::invalid_argument("input segments do not form a polygon!");
    }
#endif
}

Polygon::Polygon(const std::initializer_list<Segment>& _segments) : segments(_segments)
{
#ifdef VALIDATE_POLYGON
    if (!isValid())
    {
        throw std::invalid_argument("input segments do not form a polygon!");
    }
#endif
}

bool Polygon::containsPoint(const Point& point)
{
    // cast a ray from the point in the +x direction
    Ray ray(point, Vector(1, 0));
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
    return num_intersections % 2 == 0;
}

bool Polygon::intersects(const Segment& segment)
{
    for (const auto& seg : segments)
    {
        if (::intersects(seg, segment))
            return true;
    }
    return false;
}

bool Polygon::intersects(const Ray& ray)
{
    for (const auto& seg : segments)
    {
        if (::intersects(seg, ray))
            return true;
    }
    return false;
}

bool Polygon::isValid()
{
    // to have a valid polygon, each vertex should appear once in the
    // first part of a segment, and once in the 2nd part of a segment
    std::unordered_set<Point> first_point_set;
    for (const auto& seg : segments)
    {
        first_point_set.insert(seg.getSegStart());
    }
    for (const auto& seg : segments)
    {
        if (first_point_set.find(seg.getEnd()) == first_point_set.end())
        {
            // 2nd point does not occur in set of first points,
            return false;
        }
    }
    return true;
}

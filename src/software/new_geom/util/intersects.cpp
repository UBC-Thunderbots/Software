#include "software/new_geom/util/intersects.h"

#include <limits>

#include "software/new_geom/util/contains.h"
#include "software/new_geom/util/distance.h"
#include "software/new_geom/util/intersection.h"

bool intersectsNew(const Polygon &first, const Segment &second)
{
    for (const auto &seg : first.getSegments())
    {
        if (intersectsNew(seg, second))
        {
            return true;
        }
    }
    return false;
}

bool intersectsNew(const Segment &first, const Polygon &second)
{
    return intersectsNew(second, first);
}

bool intersectsNew(const Polygon &first, const Ray &second)
{
    for (const auto &seg : first.getSegments())
    {
        if (intersectsNew(seg, second))
        {
            return true;
        }
    }
    return false;
}

bool intersectsNew(const Ray &first, const Polygon &second)
{
    return intersectsNew(second, first);
}

bool intersectsNew(const Polygon &first, const Circle &second)
{
    if (first.contains(second.getOrigin()))
    {
        return true;
    }
    for (const auto &seg : first.getSegments())
    {
        if (distance(seg, second.getOrigin()) < second.getRadius())
        {
            return true;
        }
    }
    return false;
}

bool intersectsNew(const Circle &first, const Polygon &second)
{
    return intersectsNew(second, first);
}

bool intersectsNew(const Circle &first, const Circle &second)
{
    return (first.getOrigin() - second.getOrigin()).length() <
           (first.getRadius() + second.getRadius());
}

bool intersectsNew(const Segment &first, const Circle &second)
{
    bool segment_inside_circle = containsNew(second, first);
    double segment_start_circle_origin_distsq =
        distanceSquared(first.getSegStart(), second.getOrigin());
    double segment_end_circle_origin_distsq =
        distanceSquared(first.getEnd(), second.getOrigin());

    // if the segment is inside the circle AND at least one of the points is
    // outside the circle
    return segment_inside_circle &&
           (segment_start_circle_origin_distsq >
                second.getRadius() * second.getRadius() ||
            segment_end_circle_origin_distsq > second.getRadius() * second.getRadius());
}

bool intersectsNew(const Circle &first, const Segment &second)
{
    return intersectsNew(second, first);
}

bool intersectsNew(const Segment &first, const Segment &second)
{
    boost::geometry::model::segment<Point> AB(first.getSegStart(), first.getEnd());
    boost::geometry::model::segment<Point> CD(second.getSegStart(), second.getEnd());

    return boost::geometry::intersects(AB, CD);
}

bool intersectsNew(const Ray &first, const Segment &second)
{
    auto intersectionValue =
        intersection(first.getStart(), first.getStart() + first.toUnitVector(),
                     second.getSegStart(), second.getEnd());
    // If the infinitely long vectors defined by ray and segment intersect, check that the
    // intersection is within their definitions
    if (intersectionValue.has_value())
    {
        return first.contains(intersectionValue.value()) &&
               second.contains(intersectionValue.value());
    }
    // If there is no intersection, the ray and segment may be parallel, check if they are
    // overlapped
    return first.contains(second.getSegStart()) || first.contains(second.getEnd());
}

bool intersectsNew(const Segment &first, const Ray &second)
{
    return intersectsNew(second, first);
}

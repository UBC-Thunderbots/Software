#include "software/new_geom/util/intersects.h"

#include "software/new_geom/util/contains.h"
#include "software/new_geom/util/distance.h"
#include "software/new_geom/util/intersection.h"

bool intersects(const Polygon &first, const Segment &second)
{
    return first.contains(second.getSegStart()) || first.contains(second.getEnd());
}

bool intersects(const Segment &first, const Polygon &second)
{
    return intersects(second, first);
}

bool intersects(const Polygon &first, const Ray &second)
{
    for (const auto &seg : first.getSegments())
    {
        if (intersects(seg, second))
        {
            return true;
        }
    }
    return false;
}

bool intersects(const Ray &first, const Polygon &second)
{
    return intersects(second, first);
}

bool intersects(const Polygon &first, const Circle &second)
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

bool intersects(const Circle &first, const Polygon &second)
{
    return intersects(second, first);
}

bool intersects(const Circle &first, const Circle &second)
{
    return (first.getOrigin() - second.getOrigin()).length() <
           (first.getRadius() + second.getRadius());
}

bool intersects(const Segment &first, const Circle &second)
{
    double segment_start_circle_origin_distsq =
        distanceSquared(first.getSegStart(), second.getOrigin());
    double segment_end_circle_origin_distsq =
        distanceSquared(first.getEnd(), second.getOrigin());

    bool segment_inside_circle = containsNew(second, first);
    double segment_outside_circle =
        (segment_start_circle_origin_distsq > second.getRadius() * second.getRadius() ||
         segment_end_circle_origin_distsq > second.getRadius() * second.getRadius());

    return segment_inside_circle && segment_outside_circle;
}

bool intersects(const Circle &first, const Segment &second)
{
    return intersects(second, first);
}

bool intersects(const Segment &first, const Segment &second)
{
    boost::geometry::model::segment<Point> AB(first.getSegStart(), first.getEnd());
    boost::geometry::model::segment<Point> CD(second.getSegStart(), second.getEnd());

    return boost::geometry::intersects(AB, CD);
}

bool intersects(const Ray &first, const Segment &second)
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

bool intersects(const Segment &first, const Ray &second)
{
    return intersects(second, first);
}

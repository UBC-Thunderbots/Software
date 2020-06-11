#include "software/new_geom/util/intersects.h"

#include "software/new_geom/util/contains.h"
#include "software/new_geom/util/distance.h"
#include "software/new_geom/util/intersection.h"

bool intersects(const Polygon &first, const Segment &second)
{
    for (const auto &seg : first.getSegments())
    {
        if (seg.intersects(second))
        {
            return true;
        }
    }
    if (first.contains(second.getStart()))
    {
        return true;
    }
    return false;
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
    if (distance(first, second.getOrigin()) <= second.getRadius())
    {
        return true;
    }

    return false;
}

bool intersects(const Circle &first, const Segment &second)
{
    return intersects(second, first);
}

bool intersects(const Ray &first, const Segment &second)
{
    auto intersectionValue =
        intersection(first.getStart(), first.getStart() + first.toUnitVector(),
                     second.getStart(), second.getEnd());
    // If the infinitely long vectors defined by ray and segment intersect, check that the
    // intersection is within their definitions
    if (intersectionValue.has_value())
    {
        return first.contains(intersectionValue.value()) &&
               second.contains(intersectionValue.value());
    }
    // If there is no intersection, the ray and segment may be parallel, check if they are
    // overlapped
    return first.contains(second.getStart()) || first.contains(second.getEnd());
}

bool intersects(const Segment &first, const Ray &second)
{
    return intersects(second, first);
}

#include "software/new_geom/util/intersects.h"

#include "software/new_geom/util/contains.h"
#include "software/new_geom/util/distance.h"
#include "software/new_geom/util/intersection.h"

bool intersects(const Polygon &first, const Segment &second)
{
    for (const auto &seg : first.getSegments())
    {
        if (intersects(seg, second))
        {
            return true;
        }
    }
    if (first.contains(second.getSegStart()))
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

bool intersects(const Segment &first, const Segment &second)
{
    // Using the FASTER LINE SEGMENT INTERSECTION algorithm from p.199 of Graphics Gems
    // III (IBM Version)
    // https://www.sciencedirect.com/science/article/pii/B9780080507552500452.

    // Values are pre-computed to improve performance
    const double p1x(first.getSegStart().x());
    const double p1y(first.getSegStart().y());
    const double p2x(first.getEnd().x());
    const double p2y(first.getEnd().y());
    const double p3x(second.getEnd().x());
    const double p3y(second.getEnd().y());
    const double p4x(second.getSegStart().x());
    const double p4y(second.getSegStart().y());

    const double ax = p2x - p1x;
    const double ay = p2y - p1y;
    const double bx = p3x - p4x;
    const double by = p3y - p4y;
    const double cx = p1x - p3x;
    const double cy = p1y - p3y;

    double denominator = ay * bx - ax * by;
    double numerator1  = by * cx - bx * cy;
    if (denominator > 0)
    {
        if (numerator1 < 0 || numerator1 > denominator)
        {
            return false;
        }
    }
    else
    {
        if (numerator1 > 0 || numerator1 < denominator)
        {
            return false;
        }
    }

    // Only compute numerator2 once we're sure we need it
    double numerator2 = ax * cy - ay * cx;
    if (denominator > 0)
    {
        if (numerator2 < 0 || numerator2 > denominator)
        {
            return false;
        }
    }
    else
    {
        if (numerator2 > 0 || numerator2 < denominator)
        {
            return false;
        }
    }

    return true;
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

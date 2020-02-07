#include "software/new_geom/util/intersects.h"
#include "software/new_geom/util/contains.h"
#include "software/new_geom/util/intersection.h"

bool intersects(const Polygon& poly, const Segment& segment)
{
    for (const auto& seg : poly.getSegments())
    {
        if (intersects(seg, segment))
        {
            return true;
        }
    }
    return false;
}

bool intersects(const Polygon& poly, const Ray& ray)
{
    for (const auto& seg : poly.getSegments())
    {
        if (intersects(ray, seg))
        {
            return true;
        }
    }
    return false;
}

bool intersects(const Segment &first, const Segment &second)
{
    boost::geometry::model::segment<Point> seg_1(first.getSegStart(), first.getEnd());
    boost::geometry::model::segment<Point> seg_2(second.getSegStart(), second.getEnd());

    return boost::geometry::intersects(seg_1, seg_2);
}

bool intersects(const Ray &first, const Segment &segment)
{
    auto point_of_intersection = intersection(Line(first.getStart(), first.getStart() + first.toUnitVector()),
            Line(segment.getSegStart(), segment.getEnd()));

    // If the lines defined by ray and segment intersect, check that the
    // intersection is within their definitions
    if (point_of_intersection.has_value())
    {
        return contains(first, point_of_intersection.value()) && contains(segment, point_of_intersection.value());
    }
    // Otherwise, the ray and segment may be parallel, check if they are overlapping
    else
    {
        return contains(segment, first.getStart());
    }
}

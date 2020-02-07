#include "software/new_geom/util/intersects.h"

bool intersects(const Polygon& poly, const Segment& segment)
{
    for (const auto& seg : poly.getSegments())
    {
        if (::intersects(seg, segment))
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
        if (::intersects(seg, ray))
        {
            return true;
        }
    }
    return false;
}

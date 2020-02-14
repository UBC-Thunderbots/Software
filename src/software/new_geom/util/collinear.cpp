#include "software/new_geom/util/collinear.h"

bool collinear(const Point &a, const Point &b, const Point &c)
{
    if ((a - b).lengthSquared() < GeomConstants::EPSILON ||
        (b - c).lengthSquared() < GeomConstants::EPSILON ||
        (a - c).lengthSquared() < GeomConstants::EPSILON)
    {
        return true;
    }

    return std::fabs((b - a).cross(c - a)) < GeomConstants::EPSILON;
}

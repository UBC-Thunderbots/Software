#include "software/new_geom/util/collinear.h"
#include "software/new_geom/util/almost_equal.h"

bool collinear(const Point &a, const Point &b, const Point &c)
{
    if ((a - b).lengthSquared() < GeomConstants::EPSILON ||
        (b - c).lengthSquared() < GeomConstants::EPSILON ||
        (a - c).lengthSquared() < GeomConstants::EPSILON)
    {
        return true;
    }

    Vector v1 = b - a;
    Vector v2 = c - a;
    return almostEqual(v1.x() * v2.y(), v1.y() * v2.x());
}

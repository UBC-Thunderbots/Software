#include "software/new_geom/util/collinear.h"

#include "software/new_geom/util/almost_equal.h"

bool collinear(const Point &a, const Point &b, const Point &c)
{
    if ((a - b).lengthSquared() < GeomConstants::FIXED_EPSILON ||
        (b - c).lengthSquared() < GeomConstants::FIXED_EPSILON ||
        (a - c).lengthSquared() < GeomConstants::FIXED_EPSILON)
    {
        return true;
    }

    Vector v1 = b - a;
    Vector v2 = c - a;
    return almostEqual(v1.x() * v2.y(), v1.y() * v2.x(), GeomConstants::FIXED_EPSILON,
                       GeomConstants::ULPS_EPSILON_TEN);
}

#include "software/new_geom/util/collinear.h"

#include "software/new_geom/util/almost_equal.h"


bool collinear(const Point &a, const Point &b, const Point &c, double fixed_epsilon,
               int ulps_epsilon)
{
    if ((a - b).lengthSquared() < fixed_epsilon ||
        (b - c).lengthSquared() < fixed_epsilon ||
        (a - c).lengthSquared() < fixed_epsilon)
    {
        return true;
    }

    if ((almostEqual(a.x(), b.x(), fixed_epsilon, ulps_epsilon) &&
         almostEqual(a.x(), c.x(), fixed_epsilon, ulps_epsilon)) ||
        (almostEqual(a.y(), b.y(), fixed_epsilon, ulps_epsilon) &&
         almostEqual(a.y(), c.y(), fixed_epsilon, ulps_epsilon)))
    {
        // Explicit check for the vectors being near vertical or horizontal to avoid near
        // zero comparisons
        return true;
    }

    Vector v1 = b - a;
    Vector v2 = c - a;
    return almostEqual(v1.x() * v2.y(), v1.y() * v2.x(), fixed_epsilon, ulps_epsilon);
}

bool collinear(const Segment &segment1, const Segment &segment2)
{
    // Two segments are collinear if all Points are collinear
    if (collinear(segment1.getSegStart(), segment1.getEnd(), segment2.getSegStart()) &&
        collinear(segment1.getSegStart(), segment1.getEnd(), segment2.getEnd()))
    {
        return true;
    }
    return false;
}

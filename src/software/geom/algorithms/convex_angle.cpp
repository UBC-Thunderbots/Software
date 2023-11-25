#include "software/geom/algorithms/convex_angle.h"

Angle convexAngle(const Vector& v1, const Vector& v2)
{
    return v1.orientation().minDiff(v2.orientation());
}

Angle convexAngle(const Point& p1, const Point& p2, const Point& p3)
{
    return convexAngle(p1 - p2, p3 - p2);
}

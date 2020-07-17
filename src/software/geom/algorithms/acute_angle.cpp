#include "software/geom/algorithms/acute_angle.h"

Angle acuteAngle(const Vector& v1, const Vector& v2)
{
    return v1.orientation().minDiff(v2.orientation());
}

Angle acuteAngle(const Point& p1, const Point& p2, const Point& p3)
{
    return acuteAngle(p1 - p2, p3 - p2);
}

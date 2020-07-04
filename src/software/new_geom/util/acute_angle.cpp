#include "software/new_geom/util/acute_angle.h"

Angle acuteAngle(Vector v1, Vector v2)
{
    return v1.orientation().minDiff(v2.orientation());
}

Angle acuteAngle(Point p1, Point p2, Point p3)
{
    return acuteAngle(p1 - p2, p3 - p2);
}

#include "software/geom/algorithms/collinear.h"

#include "software/geom/algorithms/acute_angle.h"
#include "software/geom/algorithms/almost_equal.h"


bool collinear(const Point &a, const Point &b, const Point &c, double fixed_epsilon,
               int ulps_epsilon)
{
    return (almostEqual(acuteAngle(b - a, c - a).toRadians(), 0, fixed_epsilon,
                        ulps_epsilon) ||
            almostEqual(acuteAngle(a - b, c - b).toRadians(), 0, fixed_epsilon,
                        ulps_epsilon) ||
            almostEqual(acuteAngle(a - c, b - c).toRadians(), 0, fixed_epsilon,
                        ulps_epsilon));
}

bool collinear(const Segment &segment1, const Segment &segment2)
{
    // Two segments are collinear if all Points are collinear
    if (collinear(segment1.getStart(), segment1.getEnd(), segment2.getStart()) &&
        collinear(segment1.getStart(), segment1.getEnd(), segment2.getEnd()))
    {
        return true;
    }
    return false;
}

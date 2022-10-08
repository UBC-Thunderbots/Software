#include "software/geom/algorithms/collinear.h"

#include "software/geom/algorithms/almost_equal.h"
#include "software/geom/algorithms/convex_angle.h"


bool collinear(const Point &a, const Point &b, const Point &c)
{
    // NOTE: the default value is 4 * FIXED_EPSILON because of accumulation of error from
    // subtracting 2 vectors twice and calculating convexAngle, but the PassGenerator
    // requires less precision probably due to calcBestShotOnGoal
    // TODO (#1788): change fixed_epsilon to 4 * FIXED_EPSILON
    static const double fixed_epsilon = 100 * FIXED_EPSILON;
    static const int ulps_epsilon     = ULPS_EPSILON_TEN;
    return (almostEqual(convexAngle(b - a, c - a).toRadians(), 0, fixed_epsilon,
                        ulps_epsilon) ||
            almostEqual(convexAngle(a - b, c - b).toRadians(), 0, fixed_epsilon,
                        ulps_epsilon) ||
            almostEqual(convexAngle(a - c, b - c).toRadians(), 0, fixed_epsilon,
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

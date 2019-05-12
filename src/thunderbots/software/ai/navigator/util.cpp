#include "ai/navigator/util.h"

#include "geom/util.h"

double calculateTransitionSpeedBetweenSegments(const Point &p1, const Point &p2,
                                               const Point &p3, double final_speed)
{
    return final_speed * (p2 - p1).norm().project((p3 - p2).norm()).len();
}

double getPointTrespass(const Point &p1, const Point &p2, double trespass_threshold)
{
    double dist_trespass = dist(p1, p2);

    if (dist_trespass <= trespass_threshold)
        return dist_trespass;

    return 0;
}

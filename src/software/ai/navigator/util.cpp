#include "software/ai/navigator/util.h"

#include "software/geom/point.h"
#include "software/geom/util.h"

double calculateTransitionSpeedBetweenSegments(const Point &p1, const Point &p2,
                                               const Point &p3, double final_speed)
{
    return final_speed * (p2 - p1).norm().project((p3 - p2).norm()).len();
}

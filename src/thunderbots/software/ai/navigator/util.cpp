#include "ai/navigator/util.h"

double seg_vel(const Point &p1, const Point &p2, const Point &p3, double final_vel)
{
    return final_vel * (p2-p1).norm().project((p3-p2).norm()).len();
}

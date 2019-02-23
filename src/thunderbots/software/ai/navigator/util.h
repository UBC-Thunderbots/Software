#pragma once

#include "geom/util.h"
#include "geom/point.h"

/**
  * returns the segment's final velocity (signed) after travelling from p1 to p2
  * for a smooth transition to the p2 to p3 path, scaled by the ultimate final speed
 */
double seg_vel(const Point &p1, const Point &p2, const Point &p3, double final_vel);

#pragma once

#include "software/new_geom/geom_constants.h"
#include "software/new_geom/point.h"

/**
 * Returns true if the given points are collinear, false otherwise.
 *
 * @params a, b, c the given points
 * @param fixed_epsilon the epsilon value for near zero double comparisons
 * @param ulps_epsilon the epsilon value for double comparisons based on ULPs distance
 *
 * @return true if the given points are collinear, false otherwise
 */
bool collinear(const Point &a, const Point &b, const Point &c,
               double fixed_epsilon = FIXED_EPSILON, int ulps_epsilon = ULPS_EPSILON_TEN);

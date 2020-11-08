#pragma once

#include "software/geom/geom_constants.h"
#include "software/geom/point.h"
#include "software/geom/segment.h"

/**
 * Returns true if the given points are collinear, false otherwise.
 *
 * @params a, b, c the given points
 * @param fixed_epsilon the epsilon value for near zero double comparisons
 * NOTE: the default value is 4*FIXED_EPSILON because of accumulation of error from
 * subtracting 2 vectors twice and calculating acuteAngle
 * @param ulps_epsilon the epsilon value for double comparisons based on ULPs distance
 *
 * @return true if the given points are collinear, false otherwise
 */
bool collinear(const Point &a, const Point &b, const Point &c,
               double fixed_epsilon = 4 * FIXED_EPSILON,
               int ulps_epsilon     = ULPS_EPSILON_TEN);

/**
 * Checks if 2 Segments are collinear.
 *
 * @param segment1 : The first Segment
 *
 * @param segment2 : The second Segment
 *
 * @return true : If the Segment1 and Segment2 are collinear within FIXED_EPSILON disance
 *
 * @return false : If Segment1 and Segment2 are NOT collinear within FIXED_EPSILON
 * distance
 */
bool collinear(const Segment &segment1, const Segment &segment2);

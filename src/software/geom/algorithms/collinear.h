#pragma once

#include "software/geom/geom_constants.h"
#include "software/geom/point.h"
#include "software/geom/segment.h"

/**
 * Returns true if the given points are collinear, false otherwise.
 *
 * @params a, b, c the given points
 *
 * @return true if the given points are collinear, false otherwise
 */
bool collinear(const Point &a, const Point &b, const Point &c);

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

#pragma once

#include "geom/point.h"
#include "geom/util.h"

/**
 * Calculates the transition velocity for the robot between two line segments
 *
 * Calculates the velocity that the robot should be at when it is at the end of a
 * given line segment in order to smoothly transition to another given line segment,
 * given a final velocity at the end of the two line segments
 *
 * @param p1, p2, p3 are 3 points that define two line segments that form a path
 * @param final_vel is the intended final velocity at the end of the path
 * @return the first segment's final velocity (signed) after travelling from p1 to p2
 * for a smooth transition to the p2 to p3 path, scaled by the final velocity at the end
 * of the path
 */
double calculateTransitionVelocityBetweenSegments(const Point &p1, const Point &p2,
                                                  const Point &p3, double final_vel);

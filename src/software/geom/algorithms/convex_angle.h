#pragma once

#include "software/geom/point.h"

/**
 * Calculates the convex angle formed by the two given vectors
 * A convex angle is between 0 and π radians inclusive-
 * https://proofwiki.org/wiki/Definition:Convex_Angle
 *
 * @param v1
 * @param v2
 *
 * @return The convex angle formed by v1 and v2, in the range [0, π]
 */
Angle convexAngle(const Vector& v1, const Vector& v2);

/**
 * Calculates the convex angle formed by the vector p2->p1 and p2->p3
 * A convex angle is between 0 and π radians inclusive-
 * https://proofwiki.org/wiki/Definition:Convex_Angle
 *
 * @param p1
 * @param p2
 * @param p3
 *
 * @return The convex angle formed by the vector p2->p1 and p2->p3, in the range [0, π]
 */
Angle convexAngle(const Point& p1, const Point& p2, const Point& p3);

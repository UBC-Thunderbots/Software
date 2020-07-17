#pragma once

#include "software/geom/point.h"

/**
 * Calculates the acute angle formed by the two given vectors
 *
 * @param v1
 * @param v2
 *
 * @return The acute angle formed by v1 and v2
 */
Angle acuteAngle(const Vector& v1, const Vector& v2);

/**
 * Calculates the acute angle formed by the vector p2->p1 and p2->p3
 *
 * @param p1
 * @param p2
 * @param p3
 *
 * @return the acute angle formed by the vector p2->p1 and p2->p3
 */
Angle acuteAngle(const Point& p1, const Point& p2, const Point& p3);

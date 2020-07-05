#pragma once

#include "software/new_geom/point.h"

/**
 * Calculates the acute angle formed by the two given vectors
 *
 * @param v1
 * @param v2
 *
 * @return The acute angle formed by v1 and v2
 */
Angle acuteAngle(Vector v1, Vector v2);

/**
 * Calculates the acute angle formed by the vector p2->p1 and p2->p3
 *
 * @param p1
 * @param p2
 * @param p3
 *
 * @return the acute angle formed by the vector p2->p1 and p2->p3
 */
Angle acuteAngle(Point p1, Point p2, Point p3);

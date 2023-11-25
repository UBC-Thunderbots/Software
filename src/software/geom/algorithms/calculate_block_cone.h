#pragma once

#include "software/geom/point.h"
#include "software/geom/vector.h"

/**
 * Given a cone shooting from the origin, determines the furthest location from the
 * origin, at which to place a circle to block the cone formed by the vectors a and b.
 *
 * @pre The cone must have nonzero area.
 *
 * @pre \p b must be counterclockwise of \p a.
 *
 * @param a the starting angle of the cone.
 *
 * @param b the ending angle of the cone.
 *
 * @param radius the radius of the circle with which to block the cone.
 *
 * @return the blocking position.
 */
Point calculateBlockCone(const Vector &a, const Vector &b, const double radius);

/**
 * Given a cone shooting from a point P, determines the furthest location from
 * P, at which to place a circle to block the cone.
 *
 * @pre The cone must have nonzero area.
 *
 * @pre \p b must be counterclockwise of \p a.
 *
 * @param a the point such that a vector from p to a represents the right side of the
 * cone.
 *
 * @param b the point such that a vector from p to b represents the left side of the cone.
 *
 * @param radius the radius of the circle with which to block the cone.
 *
 * @param p the source of the cone.
 *
 * @return the blocking position.
 */
Point calculateBlockCone(const Point &a, const Point &b, const Point &p,
                         const double radius);

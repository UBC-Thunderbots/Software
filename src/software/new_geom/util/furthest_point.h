#pragma once

#include "software/new_geom/point.h"
#include "software/new_geom/rectangle.h"

/**
 * Computes the furthest point on `a` from any point on `b`
 * @param a
 * @param b
 * @return The furthest point on `a` from any point on `b`
 */
Point furthestPoint(const Rectangle& a, const Point& b);

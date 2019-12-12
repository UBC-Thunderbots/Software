#pragma once

#include "software/new_geom/line.h"
#include "software/new_geom/point.h"

/**
 * Finds the closest Point on a Line to a Point
 *
 * @param p the Point
 * @param l the Line
 * @return the Point closest to Point p on Line l
 */
Point closestPointOnLine(const Point &p, const Line &l);
Point closestPointOnLine(const Line &l, const Point &p);

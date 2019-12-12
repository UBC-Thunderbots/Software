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

/**
 * Finds the closest Point on a Line to a Point
 *
 * @param p the Point
 * @param p1 the first Point on the Line
 * @param p2 the second Point on the Line
 * @return the Point closest to Point p on Line formed by Points p1 and p2
 */
Point closestPointOnLine(const Point &p, const Point &p1, const Point &p2);

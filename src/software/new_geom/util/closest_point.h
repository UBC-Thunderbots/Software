#pragma once

#include "software/new_geom/line.h"
#include "software/new_geom/point.h"
#include "software/new_geom/segment.h"

/**
 * Finds the closest Point on a Line to a Point
 *
 * @param p the Point
 * @param l the Line
 * @return the Point closest to Point p on Line l
 */
Point closestPointOnLine(const Point &p, const Line &l);
Point closestPointOnLine(const Line &l, const Point &p);

/**
 * Finds the Point on line segment closest to point.
 *
 * @param p the point.
 *
 * @param segment the line segment.
 *
 * @return the Point on line segment closest to centre point.
 */
Point closestPointOnSeg(const Point &p, const Segment &segment);
Point closestPointOnSeg(const Segment &segment, const Point &p);

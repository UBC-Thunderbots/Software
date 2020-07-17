#pragma once

#include "software/geom/line.h"
#include "software/geom/point.h"
#include "software/geom/segment.h"

/**
 * Finds the closest Point on a Line to a Point
 *
 * @param p the Point
 * @param l the Line
 * @return the Point closest to Point p on Line l
 */
Point closestPoint(const Point &p, const Line &l);
Point closestPoint(const Line &l, const Point &p);

/**
 * Finds the Point on line segment closest to point.
 *
 * @param p the point.
 * @param segment the line segment.
 *
 * @return the Point on line segment closest to point.
 */
Point closestPoint(const Point &p, const Segment &segment);
Point closestPoint(const Segment &segment, const Point &p);

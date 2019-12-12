#pragma once

#include "software/new_geom/line.h"
#include "software/new_geom/point.h"

/**
 * Finds the shortest distance between a Line and a Point
 *
 * @param first the Line
 * @param second the Point
 * @return the shortest distance between first and second
 */
double distance(const Line &first, const Point &second);

/**
 * Finds the shortest distance between a Point and a Line
 *
 * @param first the Point
 * @param second the Line
 * @return the shortest distance between first and second
 */
double distance(const Point &first, const Line &second);

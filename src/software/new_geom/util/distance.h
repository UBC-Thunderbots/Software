#pragma once

#include <limits>

#include "software/geom/util.h"
#include "software/new_geom/line.h"
#include "software/new_geom/point.h"
#include "software/new_geom/polygon.h"
#include "software/new_geom/segment.h"

/**
 * Finds the shortest distance between a Line and a Point
 *
 * @param first the Line
 * @param second the Point
 * @return the shortest distance between first and second
 */
double distance(const Line &first, const Point &second);
double distance(const Point &first, const Line &second);

/**
 * Finds the shortest distance between two Points
 *
 * @param first Point
 * @param second Point
 * @return the shortest distance between first and second
 */
double distance(const Point &first, const Point &second);

/**
 * Finds the shortest distance between two Segments
 *
 * @param first Segment
 * @param second Segment
 * @return the shortest distance between first and second
 */
double distance(const Segment &first, const Segment &second);

/**
 * Finds the shortest distance between a Point and a Segment
 *
 * @param first the Point
 * @param second the Segment
 * @return the shortest distance between first and second
 */
double distance(const Point &first, const Segment &second);
double distance(const Segment &first, const Point &second);

/**
 * Finds the shortest distance between a Point and a Polygon
 *
 * @param first the Point
 * @param second the Polygon
 * @return the shortest distance between first and second
 */
double distance(const Point &first, const Polygon &second);
double distance(const Polygon &first, const Point &second);

/**
 * Finds the squared shortest distance between a Point and a Segment
 *
 * @param first the Point
 * @param second the Segment
 * @return the squared shortest distance between first and second
 */
double distanceSquared(const Point &first, const Segment &second);
double distanceSquared(const Segment &first, const Point &second);

/**
 * Finds the squared shortest distance between two Points
 *
 * @param first Point
 * @param second Point
 * @return the squared shortest distance between first and second
 */
double distanceSquared(const Point &first, const Point &second);

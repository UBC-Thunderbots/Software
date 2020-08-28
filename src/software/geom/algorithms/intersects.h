#pragma once

#include "software/geom/circle.h"
#include "software/geom/polygon.h"
#include "software/geom/ray.h"
#include "software/geom/segment.h"

/**
 * Set of functions that determines if two geometric objects intersect,
 * where intersects is defined as having at least a common point
 *
 * https://en.wikipedia.org/wiki/Spatial_relation#Topological_relations
 */

/**
 * Returns whether the segment intersects the polygon
 * A segment is considered to intersect the polygon if a portion (but not all) of the
 * segment is inside the polygon
 *
 * @param first
 * @param second
 * @return true if the segment intersects the polygon, false otherwise
 */
bool intersects(const Polygon &first, const Segment &second);
bool intersects(const Segment &first, const Polygon &second);

/**
 * Returns true if the ray intersects the polygon, false otherwise.
 *
 * @param first
 * @param second
 * @return true if the ray intersects the polygon, false otherwise
 */
bool intersects(const Polygon &first, const Ray &second);
bool intersects(const Ray &first, const Polygon &second);

/**
 * Returns true if the polygon intersects the circle, false otherwise.
 *
 * @param first
 * @param second
 * @return true if the polygon intersects the circle, false otherwise
 */
bool intersects(const Polygon &first, const Circle &second);
bool intersects(const Circle &first, const Polygon &second);

/**
 * Returns true if the circles intersect each other, false otherwise
 *
 * @param first
 * @param second
 * @return true if the circles intersect each other, false otherwise
 */
bool intersects(const Circle &first, const Circle &second);

/**
 * Returns whether the segment intersects the circle
 * A segment is considered to intersect the circle if there is at least some portion of
 * the segment that is inside the circle
 *
 * @param first
 * @param second
 * @return true if the segment inersects the circle, false otherwise
 */
bool intersects(const Segment &first, const Circle &second);
bool intersects(const Circle &first, const Segment &second);

/**
 * Returns true if the segments intersect each other, false otherwise
 *
 * @param first
 * @param second
 * @return true if the segments intersect each other, false otherwise
 */
bool intersects(const Segment &first, const Segment &second);

/**
 * Returns true if the ray intersects the segment, false otherwise
 *
 * @param first
 * @param second
 * @return true if the ray intersects the segment, false otherwise
 */
bool intersects(const Ray &first, const Segment &second);
bool intersects(const Segment &first, const Ray &second);

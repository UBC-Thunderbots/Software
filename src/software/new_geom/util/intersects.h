#pragma once

#include "software/new_geom/circle.h"
#include "software/new_geom/polygon.h"
#include "software/new_geom/ray.h"
#include "software/new_geom/segment.h"

/**
 * Returns true if the segment intersects the polygon, false otherwise.
 *
 * @param first
 * @param second
 * @return true if the segment intersects the polygon, false otherwise
 */
bool intersectsNew(const Polygon &first, const Segment &second);
bool intersectsNew(const Segment &first, const Polygon &second);

/**
 * Returns true if the ray intersects the polygon, false otherwise.
 *
 * @param first
 * @param second
 * @return true if the ray intersects the polygon, false otherwise
 */
bool intersectsNew(const Polygon &first, const Ray &second);
bool intersectsNew(const Ray &first, const Polygon &second);

/**
 * Returns true if the polygon intersects the circle, false otherwise.
 *
 * @param first
 * @param second
 * @return true if the polygon intersects the circle, false otherwise
 */
bool intersectsNew(const Polygon &first, const Circle &second);
bool intersectsNew(const Circle &first, const Polygon &second);

/**
 * Returns true if the circles intersect each other, false otherwise
 *
 * @param first
 * @param second
 * @return true if the circles intersect each other, false otherwise
 */
bool intersectsNew(const Circle &first, const Circle &second);

/**
 * Returns true if the segment intersects the circle, false otherwise.
 * A segment is considered to intersect the circle iff it has one point
 * inside the circle, and one point outside the circle.
 *
 * @param first
 * @param second
 * @return true if the segment inersects the circle, false otherwise
 */
bool intersectsNew(const Segment &first, const Circle &second);
bool intersectsNew(const Circle &first, const Segment &second);

/**
 * Returns true if the segments intersect each other, false otherwise
 *
 * @param first
 * @param second
 * @return true if the segments intersect each other, false otherwise
 */
bool intersectsNew(const Segment &first, const Segment &second);

/**
 * Returns true if the ray intersects the segment, false otherwise
 *
 * @param first
 * @param second
 * @return true if the ray intersects the segment, false otherwise
 */
bool intersectsNew(const Ray &first, const Segment &second);
bool intersectsNew(const Segment &first, const Ray &second);

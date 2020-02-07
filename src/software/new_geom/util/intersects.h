#pragma once

#include "software/new_geom/ray.h"
#include "software/new_geom/line.h"
#include "software/new_geom/polygon.h"
#include "software/new_geom/segment.h"

/**
 * Returns true if the segment intersects the polygon, false otherwise.
 *
 * @param poly The polygon to see if the segment intersects it
 * @param segment The segment to see if it intersects the polygon
 * @return true if the segment intersects the polygon, false otherwise
 */
bool intersects(const Polygon& poly, const Segment& segment);

/**
 * Returns true if the ray intersects the polygon, false otherwise.
 *
 * @param poly The polygon to see if the ray intersects it
 * @param ray The ray to see if it intersects the polygon
 * @return true if the ray intersects the polygon, false otherwise
 */
bool intersects(const Polygon& poly, const Ray& ray);

/**
 * Returns true if the two segments intersect, false otherwise.
 *
 * @param first, second
 * @return true if the two segments intersect, false otherwise
 */
bool intersects(const Segment &first, const Segment &second);

/**
 * Returns true if the ray intersects the segment, false otherwise.
 *
 * @param segment The segment to see if the ray intersects it
 * @param ray The ray to see if it intersects the segment
 * @return true if the ray intersects the segment, false otherwise
 */
bool intersects(const Ray &ray, const Segment &segment);

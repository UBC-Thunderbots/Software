#pragma once

#include "software/geom/util.h"
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

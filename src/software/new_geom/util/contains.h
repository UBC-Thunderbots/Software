#include "software/new_geom/ray.h"
#include "software/new_geom/segment.h"

/**
 * Returns true if the given ray contains the given point, false otherwise.
 *
 * @param ray
 * @param point
 *
 * @return true if the given ray contains the given point, false otherwise
 */
bool contains(const Ray &ray, const Point &point);

/**
 * Returns true if the given segment contains the given point, false otherwise.
 *
 * @param segment
 * @param point
 *
 * @return true if the given segment contains the given point, false otherwise
 */
bool contains(const Segment &segment, const Point &point);

#pragma once

#include "software/new_geom/circle.h"
#include "software/new_geom/point.h"
#include "software/new_geom/ray.h"

/**
 * Returns the circle's tangent points.
 *
 * @param start The start point
 * @param circle The circle to find tangents points with
 *
 * Returns the points on the circle that form tangent lines with the start point
 */
std::pair<Point, Point> getCircleTangentPoints(const Point &start, const Circle &circle);

/**
 * Calculates the pair of Rays that intercept the Circle tangentially with origin at the
 * reference Point
 *
 * @param reference: The point which the tangent vectors will intersect
 * @param circle: The circle to calculate the tangent vectors of
 * @return the mean point of points
 */
std::pair<Ray, Ray> getCircleTangentRaysWithReferenceOrigin(const Point reference,
                                                            const Circle circle);

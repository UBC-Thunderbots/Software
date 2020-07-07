#pragma once

#include <optional>

#include "software/new_geom/circle.h"
#include "software/new_geom/point.h"
#include "software/new_geom/ray.h"
#include "software/new_geom/segment.h"

/**
 * Function returns the segment defined by the segment between the intersections of two
 * Rays on a segment
 *
 * @param ray1 (Starting point and direction)
 * @param ray2 (Starting point and direction)
 * @param segment (Segment to find segment of intersection upon)
 * @return Segment, the segment defined by the space between two intersecting rays on the
 * segment parameter std::nullopt, if both rays don't intersect the segment, and the
 * segment is not enclosed between the rays Segment, if one ray intersects the segment,
 * but one of the segment parameters extremes are enclosed within the two rays
 */
std::optional<Segment> getIntersectingSegment(Ray ray1, Ray ray2, Segment segment);

/**
 * Function calculates whether the segment parameter is enclosed between the ray
 * parameters. This means the entirety of the segment lays between the rays
 *
 * @param segment : segment parameter to calculate if its definition lies between the rays
 * @param ray1 : Starting point and direction
 * @param ray2 : Starting point an direction
 * @return Segment: Returns the segment parameter if it is completely enclosed between
 * ray1 and ray2.
 *
 * Example of segment being enclosed by rays:
 *
 *        segment
 *     \ *----*  /
 *      \       /
 *  ray1 \     /ray2
 *        *   *
 *
 * Returns std::nullopt of the ray is not completely enclosed between the rays, or
 * not at all
 */
std::optional<Segment> segmentEnclosedBetweenRays(Segment segment, Ray ray1, Ray ray2);

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

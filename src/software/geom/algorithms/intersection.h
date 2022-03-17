#pragma once

#include <optional>
#include <unordered_set>

#include "software/geom/line.h"
#include "software/geom/polygon.h"
#include "software/geom/ray.h"
#include "software/geom/segment.h"

/**
 * A set of functions that calculate the intersecting points between two geometric objects
 */

/**
 * Computes the point of intersection between two lines.
 * Note: this computes the intersection of two lines, not line segments.
 *
 * Overlapping lines are considered parallel and not intersecting.
 *
 * See:
 * https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
 *
 * @params a, b points that represent the first line
 * @params c, d points that represent the second line
 *
 * @return the point of intersection, if it exists
 */
std::optional<Point> intersection(const Point &a, const Point &b, const Point &c,
                                  const Point &d, double fixed_epsilon = FIXED_EPSILON,
                                  int ulps_epsilon = ULPS_EPSILON_TEN);

/**
 * Computes the intersection of two segments.
 *
 * @param first
 * @param second
 *
 * @return  one of:
 *          - an empty vector if no intersections
 *          - a vector containing a single point of intersection
 *          - a vector containing two points representing the line segment of the overlap
 * if both segments are collinear and overlapping
 */
std::vector<Point> intersection(const Segment &first, const Segment &second);

/**
 * Computes the points of intersection between a polygon and a line segment.
 *
 * @param polygon
 * @param segment
 *
 * @return a set containing the points of intersection between a polygon and a
 * segment
 */
std::unordered_set<Point> intersection(const Polygon &polygon, const Segment &segment);

/**
 * Computes the points of intersection between a ray and a line segment.
 *
 * @param ray
 * @param segment
 *
 * @return  one of:
 *          - an empty vector if no intersections
 *          - a vector containing a single point of intersection
 *          - a vector containing two points representing the line segment of the overlap
 * if the ray and segment are collinear and overlapping
 */
std::vector<Point> intersection(const Ray &ray, const Segment &segment);

/**
 * Computes the point of intersection between two lines.
 *
 * Overlapping lines are considered parallel and not intersecting.
 *
 * @param first
 * @param second
 *
 * @return the point of intersection between the two lines, if it exists
 */
std::optional<Point> intersection(const Line &first, const Line &second);

/**
 * Computes the points of intersection between a polygon and a ray.
 *
 * @param polygon
 * @param ray
 *
 * @return a set containing the points of intersection between a polygon and a ray
 */
std::unordered_set<Point> intersection(const Polygon &polygon, const Ray &ray);

/**
 * Computes the point of intersection between two rays.
 * Note: Returns nullopt if rays are overlapping and parallel.
 *
 * @param first
 * @param second
 *
 * @return the point of intersection, if it exists
 */
std::optional<Point> intersection(const Ray &first, const Ray &second);

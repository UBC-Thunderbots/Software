#pragma once

#include <algorithm>
#include <optional>
#include <unordered_set>
#include <vector>

#include "software/geom/circle.h"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"

/**
 * Finds all circles which do not contain a point in them within the given rectangle.
 *
 * NOTE: this only guarantees that the center of each circle is within the
 *       rectangle, some portion of the circle may extend outside the rectangle
 *
 * @param bounding_box The rectangle in which to look for open circles
 * @param points The points that must not lie within the circles
 *
 * @return A list of circles, sorted in descending order of radius. If no points were
 * provided, returns an empty list. Any points outside the bounding_box are omitted.
 */
std::vector<Circle> findOpenCircles(const Rectangle &bounding_box,
                                    std::vector<Point> points);

/**
 *
 * Finds the point in the testPoints vector that is closest to the originPoint.
 *
 * @param originPoint
 * @param testPoints
 * @return The point in testPoints closest to testPoints.
 */
std::optional<Point> findClosestPoint(const Point &origin_point,
                                      const std::vector<Point> &test_points);

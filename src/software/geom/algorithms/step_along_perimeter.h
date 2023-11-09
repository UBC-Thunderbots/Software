#pragma once

#include "software/geom/point.h"
#include "software/geom/polygon.h"


/**
 * Computes the final position on a polygon's perimeter after traveling a specified
 * distance from a starting point.
 *
 * Given a polygon, a starting point, and a travel distance, this function calculates the
 * final point reached upon traveling the distance along the perimeter starting from the
 * point on the perimeter closest to the start point. Travel direction is determined by
 * the sign of the distance: positive travels clockwise, while negative travels
 * counterclockwise.
 *
 * @note If the distance is zero, the function returns the point on the perimeter closest
 * to the starting point
 *
 * @param polygon The polygon to traverse
 * @param start The starting position on the polygon's perimeter
 * @param distance The distance to traverse along the polygon's perimeter.
 * Positive to traverse clockwise, or negative to traverse counter-clockwise.
 * @return The final position after traveling the specified distance from the start
 */
Point stepAlongPerimeter(const Polygon& polygon, const Point& start,
                         double travel_distance);

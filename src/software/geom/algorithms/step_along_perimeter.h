#pragma once

#include "software/geom/polygon.h"
#include "software/geom/point.h"


/**
 * @brief Computes the final position on a polygon's perimeter after traveling a specified distance from a starting point.
 *
 * Given a polygon, a starting point on its perimeter, and a travel distance,
 * this function calculates the final point reached upon traveling the distance along the perimeter.
 * Travel direction is determined by the sign of the distance: positive travels clockwise, while negative travels counterclockwise.
 *
 * @note Assumes that the starting point is exactly on the polygon's perimeter. If distance is zero, the function returns the starting point.
 *
 * @param polygon Reference to a Polygon object representing the polygon to traverse.
 * @param start Reference to a Point object representing the starting position on the polygon's perimeter.
 * @param distance The distance to traverse along the polygon's perimeter. Can be negative.
 * @return Point object representing the final position after traveling the specified distance from the start.
 */
Point stepAlongPerimeter(const Polygon& polygon, const Point& start, double distance);



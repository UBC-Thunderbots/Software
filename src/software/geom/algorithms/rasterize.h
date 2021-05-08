#pragma once

#include <vector>
#include "software/geom/circle.h"
#include "software/geom/polygon.h"
#include "software/geom/rectangle.h"

/**
 * Set of functions that rasterize a geometric object into a set of points
 *
 * https://
 */

/**
 * Returns a set of points covered by a circle with a set resolution size
 *
 * @param circle circle being converted into a set of points
 * @param resolution_size how far should the points be apart from each other in meters
 * @return a set of points covered by circle
 */
std::vector<Point> rasterize(const Circle &circle, double resolution_size);

/**
 * Returns a set of points covered by a circle with a set resolution size
 *
 * @param rectangle rectangle being converted into a set of points
 * @param resolution_size how far should the points be apart from each other in meters
 * @return a set of points covered by rectangle
 */
std::vector<Point> rasterize(const Rectangle &rectangle, double resolution_size);

/**
 * Returns a set of points covered by a polygon with a set resolution size
 *
 * @param polygon polygon being converted into a set of points
 * @param resolution_size how far should the points be apart from each other in meters
 * @return a set of points covered by polygon
 */
std::vector<Point> rasterize(const Polygon &polygon, double resolution_size);

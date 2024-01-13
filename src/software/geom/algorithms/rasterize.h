#pragma once

#include <vector>

#include "software/geom/circle.h"
#include "software/geom/polygon.h"
#include "software/geom/rectangle.h"
#include "software/geom/stadium.h"

/**
 * Set of functions that rasterize a geometric object into a set of points
 */

/**
 * Returns a set of points covered by a circle with a set resolution size
 * Using x^2 + y^2 = r^2
 *
 * @param circle circle being converted into a set of points
 * @param resolution_size how far should the points be apart from each other in meters
 * @return a set of points covered by circle
 */
std::vector<Point> rasterize(const Circle &circle, double resolution_size);

/**
 * Returns a set of points covered by a rectangle with a set resolution size
 * NOTE: the set of points are not guaranteed to be distributed equally throughout the
 * rectangle if the rectangle width and height are not divisible by resolution_size
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

/**
 * Returns a set of points covered by a stadium with a set resolution size
 *
 * @param stadium stadium being converted into a set of points
 * @param resolution_size how far should the points be apart from each other in meters
 * @return a set of points covered by stadium
 */
std::vector<Point> rasterize(const Stadium &stadium, double resolution_size);
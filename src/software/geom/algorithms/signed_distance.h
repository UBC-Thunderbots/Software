#pragma once

#include <limits>

#include "software/geom/circle.h"
#include "software/geom/point.h"
#include "software/geom/polygon.h"
#include "software/geom/rectangle.h"
#include "software/geom/stadium.h"

/**
 * Finds the shortest distance between the perimeter of a Rectangle and a Point.
 * If the point is inside the Rectangle then the distance is negative, outside is
 * positive. See https://iquilezles.org/articles/distfunctions2d/ for details on the maths
 *
 * @param first
 * @param second
 * @return the signed shortest distance between the perimeter of the first and the second
 */
double signedDistance(const Rectangle &first, const Point &second);
double signedDistance(const Point &first, const Rectangle &second);

/**
 * Finds the shortest distance between the perimeter of a Circle and a Point.
 * If the point is inside the Circle then the distance is negative, outside is positive.
 * See https://iquilezles.org/articles/distfunctions2d/ for details on the maths
 *
 * @param first
 * @param second
 * @return the signed shortest distance between the perimeter of the first and the second
 */
double signedDistance(const Circle &first, const Point &second);
double signedDistance(const Point &first, const Circle &second);

/**
 * Finds the shortest distance between the perimeter of a Polygon and a Point.
 * If the point is inside the Polygon then the distance is negative, outside is positive.
 * See https://iquilezles.org/articles/distfunctions2d/ for details on the maths
 *
 * @param first
 * @param second
 * @return the signed shortest distance between the perimeter of the first and the second
 */
double signedDistance(const Polygon &first, const Point &second);
double signedDistance(const Point &first, const Polygon &second);

/**
 * Finds the shortest distance between the perimeter of a Stadium and a Point.
 * If the point is inside the Stadium then the distance is negative, outside is positive.
 * See https://iquilezles.org/articles/distfunctions2d/ for details on the maths
 *
 * @param first
 * @param second
 * @return the signed shortest distance between the perimeter of the first and the second
 */
double signedDistance(const Stadium &first, const Point &second);
double signedDistance(const Point &first, const Stadium &second);

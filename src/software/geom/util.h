#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "software/new_geom/circle.h"
#include "software/new_geom/line.h"
#include "software/new_geom/point.h"
#include "software/new_geom/polygon.h"
#include "software/new_geom/ray.h"
#include "software/new_geom/rectangle.h"
#include "software/new_geom/segment.h"
#include "software/new_geom/triangle.h"

constexpr int sign(double n)
{
    return n > FIXED_EPSILON ? 1 : (n < -FIXED_EPSILON ? -1 : 0);
}

double length(const Segment &segment);

double lengthSquared(const Segment &segment);

/**
 * Finds the Point on line segment closest to point.
 *
 * @param centre the point.
 *
 * @param segA one end of the line segment.
 *
 * @param segB the other end of the line segment.
 *
 * @return the Point on line segment closest to centre point.
 */
Point closestPointOnSeg(const Point &centre, const Point &segA, const Point &segB);
Point closestPointOnSeg(const Point &p, const Segment &segment);

/**
 * returns perpendicular offset from line x0-x1 to point p
 */
double offsetToLine(Point x0, Point x1, Point p);

/**
 * Returns the mean of a list of points
 *
 * @param points the vector of points
 * @return the mean point of points
 */
Point getPointsMean(const std::vector<Point> &points);

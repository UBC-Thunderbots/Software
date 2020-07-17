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

/**
 * returns perpendicular offset from line x0-x1 to point p
 */
double offsetToLine(Point x0, Point x1, Point p);

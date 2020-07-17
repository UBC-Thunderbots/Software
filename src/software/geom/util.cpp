#include "software/geom/util.h"

#include <algorithm>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <tuple>

#include "software/logger/logger.h"
#include "software/new_geom/angle.h"
#include "software/new_geom/rectangle.h"
#include "software/new_geom/triangle.h"
#include "software/new_geom/util/contains.h"
#include "software/new_geom/util/distance.h"
#include "software/new_geom/util/intersection.h"
#include "software/new_geom/util/intersects.h"

bool uniqueLineIntersects(const Point &a, const Point &b, const Point &c, const Point &d)
{
    return std::abs((d - c).cross(b - a)) > FIXED_EPSILON;
}

double offsetToLine(Point x0, Point x1, Point p)
{
    Vector n;

    // get normal to line
    n = (x1 - x0).perpendicular().normalize();

    return fabs(n.dot(p - x0));
}

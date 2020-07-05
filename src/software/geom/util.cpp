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
#include "software/new_geom/segment.h"
#include "software/new_geom/triangle.h"
#include "software/new_geom/util/contains.h"
#include "software/new_geom/util/distance.h"
#include "software/new_geom/util/intersection.h"
#include "software/new_geom/util/intersects.h"

bool isDegenerate(const Segment &segment)
{
    return distanceSquared(segment.getSegStart(), segment.getEnd()) <
           FIXED_EPSILON * FIXED_EPSILON;
}

double length(const Segment &segment)
{
    return distance(segment.getSegStart(), segment.getEnd());
}

double lengthSquared(const Segment &segment)
{
    return distanceSquared(segment.getSegStart(), segment.getEnd());
}

Point closestPointOnSeg(const Point &p, const Segment &segment)
{
    return closestPointOnSeg(p, segment.getSegStart(), segment.getEnd());
}
Point closestPointOnSeg(const Point &centre, const Point &segA, const Point &segB)
{
    // if one of the end-points is extremely close to the centre point
    // then return 0.0
    if ((segB - centre).lengthSquared() < FIXED_EPSILON * FIXED_EPSILON)
    {
        return segB;
    }

    if ((segA - centre).lengthSquared() < FIXED_EPSILON * FIXED_EPSILON)
    {
        return segA;
    }

    // take care of 0 length segments
    if ((segB - segA).lengthSquared() < FIXED_EPSILON * FIXED_EPSILON)
    {
        return segA;
    }

    // find point C
    // which is the projection onto the line
    double lenseg = (segB - segA).dot(centre - segA) / (segB - segA).length();
    Point C       = segA + lenseg * (segB - segA).normalize();

    // check if C is in the line seg range
    double AC     = (segA - C).lengthSquared();
    double BC     = (segB - C).lengthSquared();
    double AB     = (segA - segB).lengthSquared();
    bool in_range = AC <= AB && BC <= AB;

    // if so return C
    if (in_range)
    {
        return C;
    }
    double lenA = (centre - segA).length();
    double lenB = (centre - segB).length();

    // otherwise return closest end of line-seg
    if (lenA < lenB)
    {
        return segA;
    }
    return segB;
}

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

Point getPointsMean(const std::vector<Point> &points)
{
    Point average = Point(0, 0);
    for (unsigned int i = 0; i < points.size(); i++)
    {
        average += points[i].toVector();
    }

    Vector averageVector = average.toVector();

    averageVector /= static_cast<double>(points.size());
    return Point(averageVector);
}

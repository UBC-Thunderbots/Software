#include "software/new_geom/util/closest_point.h"

Point closestPointOnLine(const Point &p, const Line &l)
{
    Line::Coeffs coeffs = l.getCoeffs();
    double denominator  = pow(coeffs.a, 2) + pow(coeffs.b, 2);
    double x = (coeffs.b * (coeffs.b * p.x() - coeffs.a * p.y()) - coeffs.a * coeffs.c) /
               denominator;
    double y = (coeffs.a * (-coeffs.b * p.x() + coeffs.a * p.y()) - coeffs.b * coeffs.c) /
               denominator;
    return Point(x, y);
}

Point closestPointOnLine(const Line &l, const Point &p)
{
    return closestPointOnLine(p, l);
}


Point closestPointOnSeg(const Point &p, const Segment &segment)
{
    return closestPointOnSeg(p, segment.getStart(), segment.getEnd());
}

Point closestPointOnSeg(const Segment &segment, const Point &p)
{
    return closestPointOnSeg(p, segment.getStart(), segment.getEnd());
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

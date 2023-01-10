#include "software/geom/algorithms/closest_point.h"

Point closestPoint(const Point &p, const Line &l)
{
    Line::Coeffs coeffs = l.getCoeffs();
    double denominator  = pow(coeffs.a, 2) + pow(coeffs.b, 2);
    double x = (coeffs.b * (coeffs.b * p.x() - coeffs.a * p.y()) - coeffs.a * coeffs.c) /
               denominator;
    double y = (coeffs.a * (-coeffs.b * p.x() + coeffs.a * p.y()) - coeffs.b * coeffs.c) /
               denominator;
    return Point(x, y);
}

Point closestPoint(const Line &l, const Point &p)
{
    return closestPoint(p, l);
}

Point closestPoint(const Point &p, const Segment &segment)
{
    // if one of the end-points is extremely close to the centre point
    // then return 0.0
    if ((segment.getEnd() - p).lengthSquared() < FIXED_EPSILON * FIXED_EPSILON)
    {
        return segment.getEnd();
    }

    if ((segment.getStart() - p).lengthSquared() < FIXED_EPSILON * FIXED_EPSILON)
    {
        return segment.getStart();
    }

    // take care of 0 length segments
    if ((segment.getEnd() - segment.getStart()).lengthSquared() <
        FIXED_EPSILON * FIXED_EPSILON)
    {
        return segment.getStart();
    }

    // find point C
    // which is the projection onto the line
    double lenseg = (segment.getEnd() - segment.getStart()).dot(p - segment.getStart()) /
                    (segment.getEnd() - segment.getStart()).length();
    Point C =
        segment.getStart() + lenseg * (segment.getEnd() - segment.getStart()).normalize();

    // check if C is in the line seg range
    double AC     = (segment.getStart() - C).lengthSquared();
    double BC     = (segment.getEnd() - C).lengthSquared();
    double AB     = (segment.getStart() - segment.getEnd()).lengthSquared();
    bool in_range = AC <= AB && BC <= AB;

    // if so return C
    if (in_range)
    {
        return C;
    }
    double lenA = (p - segment.getStart()).length();
    double lenB = (p - segment.getEnd()).length();

    // otherwise return closest end of line-seg
    if (lenA < lenB)
    {
        return segment.getStart();
    }
    return segment.getEnd();
}

Point closestPoint(const Segment &segment, const Point &p)
{
    return closestPoint(p, segment);
}

Point closestPoint(const Polygon& polygon, const Point &p)
{
    Point closest_point;
    double closest_point_dist_sq = std::numeric_limits<double>::max();
    for (const Segment &segment : polygon.getSegments())
    {
        Point curr_closest_point = closestPoint(segment, p);
        double curr_closest_point_dist_sq = (curr_closest_point - p).lengthSquared();
        if (curr_closest_point_dist_sq < closest_point_dist_sq)
        {
            closest_point_dist_sq = curr_closest_point_dist_sq;
            closest_point = curr_closest_point;
        }
    }
    return closest_point;
}

Point closestPoint(const Circle& circle, const Point &p)
{
    Vector v = p - circle.origin();
    return circle.origin() + v.normalize(circle.radius());
}
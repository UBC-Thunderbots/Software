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

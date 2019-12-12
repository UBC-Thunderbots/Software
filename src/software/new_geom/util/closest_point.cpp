#include "software/new_geom/util/closest_point.h"

Point closestPointOnLine(const Point &p, const Line &l)
{
    std::array<double, 3> coeffs = l.getCoeffs();
    double denominator           = pow(coeffs[0], 2) + pow(coeffs[1], 2);
    double x =
        (coeffs[1] * (coeffs[1] * p.x() - coeffs[0] * p.y()) - coeffs[0] * coeffs[2]) /
        denominator;
    double y =
        (coeffs[0] * (-coeffs[1] * p.x() + coeffs[0] * p.y()) - coeffs[1] * coeffs[2]) /
        denominator;
    return Point(x, y);
}

Point closestPointOnLine(const Point &p, const Point &p1, const Point &p2)
{
    return closestPointOnLine(p, Line(p1, p2));
}

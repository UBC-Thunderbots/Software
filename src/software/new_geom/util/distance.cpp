#include "software/new_geom/util/distance.h"

double distance(const Line &first, const Point &second)
{
    Line::Coeffs coeffs = first.getCoeffs();
    return abs(coeffs.a * second.x() + coeffs.b * second.y() + coeffs.c) /
           hypot(coeffs.a, coeffs.b);
}

double distance(const Point &first, const Line &second)
{
    return distance(second, first);
}

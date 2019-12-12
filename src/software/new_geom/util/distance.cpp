#include "software/new_geom/util/distance.h"

double distance(const Line &first, const Point &second)
{
    std::array<double, 3> coeffs = first.getCoeffs();
    return abs(coeffs[0] * second.x() + coeffs[1] * second.y() + coeffs[2]) /
           hypot(coeffs[0], coeffs[1]);
}

double distance(const Point &first, const Line &second)
{
    return distance(second, first);
}

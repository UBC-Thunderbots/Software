#include "software/new_geom/line.h"

Line::Line(const Point &first, const Point &second)
{
    coeffs[0] = first.y() - second.y();
    coeffs[1] = second.x() - first.x();
    coeffs[2] = first.x() * second.y() - second.x() * first.y();
}

Vector Line::toNormalUnitVector()
{
    return Vector(coeffs[0], coeffs[1]).normalize();
}

void Line::swapXY()
{
    std::iter_swap(coeffs.begin(), coeffs.begin() + 1);
}

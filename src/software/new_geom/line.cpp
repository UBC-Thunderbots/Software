#include "software/new_geom/line.h"

Line::Line(const Point &first, const Point &second)
{
    coeffs.push_back(first.y() - second.y());
    coeffs.push_back(second.x() - first.x());
    coeffs.push_back(first.x() * second.y() - second.x() * first.y());
}

Vector Line::toNormalUnitVector()
{
    return Vector(coeffs[0], coeffs[1]).normalize();
}

void Line::swapXY()
{
    std::iter_swap(coeffs.begin(), coeffs.begin() + 1);
}

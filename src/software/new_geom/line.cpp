#include "software/new_geom/line.h"

Line::Line(const Point &first, const Point &second)
{
    if (first == second)
    {
        throw std::runtime_error("Cannot create a Line with two equal Points");
    }
    coeffs[0] = first.y() - second.y();
    coeffs[1] = second.x() - first.x();
    coeffs[2] = first.x() * second.y() - second.x() * first.y();
}

std::array<double, 3> Line::getCoeffs() const
{
    return coeffs;
}

Vector Line::toNormalUnitVector() const
{
    return Vector(coeffs[0], coeffs[1]).normalize();
}

void Line::swapXY()
{
    std::iter_swap(coeffs.begin(), coeffs.begin() + 1);
}

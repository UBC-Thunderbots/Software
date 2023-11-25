#include "software/geom/line.h"

Line::Line(const Point &first, const Point &second)
{
    if (first == second)
    {
        throw std::runtime_error("Cannot create a Line with two equal Points");
    }
    coeffs.a = first.y() - second.y();
    coeffs.b = second.x() - first.x();
    coeffs.c = first.x() * second.y() - second.x() * first.y();
}

const Line::Coeffs &Line::getCoeffs() const
{
    return coeffs;
}

Vector Line::toNormalUnitVector() const
{
    return Vector(coeffs.a, coeffs.b).normalize();
}

void Line::swapXY()
{
    double a = coeffs.a;
    coeffs.a = coeffs.b;
    coeffs.b = a;
}

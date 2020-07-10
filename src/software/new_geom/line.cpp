#include "software/new_geom/line.h"

#include <sstream>

Line::Line(const Point &first, const Point &second)
{
    if (first == second)
    {
        std::stringstream ss;
        ss << "Cannot create a Line with two equal Points: {" << first << ", " << second
           << "}";
        throw std::runtime_error(ss.str());
    }
    coeffs.a = first.y() - second.y();
    coeffs.b = second.x() - first.x();
    coeffs.c = first.x() * second.y() - second.x() * first.y();
}

Line::Coeffs Line::getCoeffs() const
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

#include "software/new_geom/line.h"

Line::Line() {}

Line::Line(double y_intercept, double slope) : Polynomial({y_intercept, slope}) {}

void Line::setCoeff(unsigned int order, double coeff)
{
    if (order > 1)
    {
        throw std::invalid_argument(
            "Tried to set the coefficient of a term with order greater than 1 for a line");
    }
    Polynomial::setCoeff(order, coeff);
};

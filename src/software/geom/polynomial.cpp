#include "software/geom/polynomial.h"

Polynomial::Polynomial(const std::vector<double>& coeffs) : coeffs(coeffs)
{
    if (coeffs.size() && (coeffs[0] == 0))
    {
        throw std::invalid_argument(
            "Size of coeffs vector isn't the order of the polynomial");
    }
}

Polynomial::Polynomial(const std::initializer_list<double>& coeffs) : coeffs(coeffs)
{
    if (this->coeffs.size() && (this->coeffs[0] == 0))
    {
        throw std::invalid_argument(
            "Size of coeffs vector isn't the order of the polynomial");
    }
}

Polynomial::Polynomial(const std::pair<double, double>& constraint1,
                       const std::pair<double, double>& constraint2)
{
    if (constraint1.first == constraint2.first)
    {
        throw std::invalid_argument("Both inputs are equal - does not define a function");
    }
    double slope = (constraint2.second - constraint1.second) /
                   (constraint2.first - constraint1.first);
    coeffs.push_back(slope);
    coeffs.push_back(constraint1.second - (constraint1.first * slope));
}

const std::vector<double>& Polynomial::getCoeffs() const
{
    return coeffs;
}

double Polynomial::valueAt(double val) const
{
    double retval = 0;
    for (size_t i = 0; i < coeffs.size(); i++)
    {
        retval += coeffs[i] * std::pow(val, coeffs.size() - 1 - i);
    }

    return retval;
}

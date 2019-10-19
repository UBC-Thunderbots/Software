#include "software/geom/polynomial.h"

Polynomial::Polynomial(const std::vector<double>& coeffs) : coeffs(coeffs)
{
    pruneCoeffs();
}

Polynomial::Polynomial(const std::initializer_list<double>& coeffs) : coeffs(coeffs)
{
    pruneCoeffs();
}

Polynomial::Polynomial(const std::pair<double, double>& p1,
                       const std::pair<double, double>& p2)
{
    double slope = (p2.second - p1.second) / (p2.first - p1.first);
    coeffs.push_back(p1.second - (p1.first * slope));
    coeffs.push_back(slope);
}

const std::vector<double>& Polynomial::getCoeffs() const
{
    return coeffs;
}

double Polynomial::calculateValue(double val) const
{
    double retval = coeffs[0];
    for (size_t i = 1; i < coeffs.size(); i++)
    {
        retval += coeffs[i] * std::pow(val, i);
    }

    return retval;
}

void Polynomial::pruneCoeffs(void)
{
    for (size_t i = coeffs.size(); i > 0; i--)
    {
        if (coeffs[i] == 0)
        {
            coeffs.pop_back();
        }
        else
        {
            break;
        }
    }
}

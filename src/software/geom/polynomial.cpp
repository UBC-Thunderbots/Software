#include "software/geom/polynomial.h"

Polynomial::Polynomial(const std::vector<double>& x_coeffs, const std::vector<double>& y_coeffs) : x_coeffs(x_coeffs), y_coeffs(y_coeffs)
{
    pruneCoeffs();
}

Polynomial::Polynomial(const std::initializer_list<double>& x_coeffs, const std::initializer_list<double>& y_coeffs) : x_coeffs(x_coeffs), y_coeffs(y_coeffs)
{
    pruneCoeffs();
}

const std::vector<double>& Polynomial::getXCoeffs() const
{
    return x_coeffs;
}

const std::vector<double>& Polynomial::getYCoeffs() const
{
    return y_coeffs;
}

Point Polynomial::calculateValue(double t) const
{
    double x_val = x_coeffs[0], y_val = y_coeffs[0];
    for (size_t i = 1; i < x_coeffs.size(); i++)
    {
        x_val += x_coeffs[i] * std::pow(t, i);
    }
    for (size_t i = 1; i < y_coeffs.size(); i++)
    {
        y_val += y_coeffs[i] * std::pow(t, i);
    }

    return Point(x_val, y_val);
}

void Polynomial::pruneCoeffs(void)
{
    for (size_t i = x_coeffs.size(); i > 0; i--)
    {
        if (x_coeffs[i] == 0)
        {
            x_coeffs.pop_back();
        }
        else
        {
            break;
        }
    }

    for (size_t i = y_coeffs.size(); i > 0; i--)
    {
        if (y_coeffs[i] == 0)
        {
            y_coeffs.pop_back();
        }
        else
        {
            break;
        }
    }
}

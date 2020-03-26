#include "software/new_geom/polynomial.h"

#include "software/new_geom/geom_constants.h"

Polynomial::Polynomial() {}

Polynomial::Polynomial(const std::vector<double> &coeffs) : coeffs(coeffs) {}

Polynomial::Polynomial(const std::initializer_list<double> &coeffs) : coeffs(coeffs) {}

double Polynomial::getCoeff(unsigned int order) const
{
    if (order >= coeffs.size())
    {
        return 0;
    }
    else
    {
        return coeffs[order];
    }
}

void Polynomial::setCoeff(unsigned int order, double coeff)
{
    if (order >= coeffs.size())
    {
        coeffs.resize(order + 1, 0);
    }
    coeffs[order] = coeff;
}

unsigned int Polynomial::getOrder() const
{
    if (coeffs.size() != 0)
    {
        for (size_t i = coeffs.size() - 1; i >= 0; i--)
        {
            if (std::abs(coeffs[i]) >= GeomConstants::FIXED_EPSILON)
            {
                return i;
            }
        }
    }
    // Zero polynomial treated as an order zero polynomial
    return 0;
}

double Polynomial::valueAt(double val) const
{
    // Horner's Method:
    // https://www.geeksforgeeks.org/horners-method-polynomial-evaluation/
    unsigned int order = getOrder();
    double retval      = getCoeff(order);
    for (unsigned int i = 1; i <= order; i++)
    {
        retval = retval * val + getCoeff(order - i);
    }
    return retval;
}

Polynomial operator+(const Polynomial &p1, const Polynomial &p2)
{
    Polynomial sum;
    unsigned int max_order = std::max(p1.getOrder(), p2.getOrder());
    for (unsigned int i = 0; i <= max_order; i++)
    {
        sum.setCoeff(i, p1.getCoeff(i) + p2.getCoeff(i));
    }
    return sum;
}

Polynomial operator-(const Polynomial &p1, const Polynomial &p2)
{
    Polynomial difference;
    unsigned int max_order = std::max(p1.getOrder(), p2.getOrder());
    for (unsigned int i = 0; i <= max_order; i++)
    {
        difference.setCoeff(i, p1.getCoeff(i) - p2.getCoeff(i));
    }
    return difference;
}

Polynomial operator*(const Polynomial &p1, const Polynomial &p2)
{
    Polynomial product;
    unsigned int p1_order = p1.getOrder();
    unsigned int p2_order = p2.getOrder();
    for (unsigned int i = 0; i <= p1_order; i++)
    {
        for (unsigned int j = 0; j <= p2_order; j++)
        {
            product.setCoeff(i + j,
                             product.getCoeff(i + j) + (p1.getCoeff(i) * p2.getCoeff(j)));
        }
    }
    return product;
}

Polynomial &operator+=(Polynomial &p1, const Polynomial &p2)
{
    return p1 = p1 + p2;
}

Polynomial &operator-=(Polynomial &p1, const Polynomial &p2)
{
    return p1 = p1 - p2;
}

Polynomial &operator*=(Polynomial &p1, const Polynomial &p2)
{
    return p1 = p1 * p2;
}

bool operator==(const Polynomial &p1, const Polynomial &p2)
{
    unsigned int p1_order = p1.getOrder();
    unsigned int p2_order = p2.getOrder();
    if (p1_order != p2_order)
    {
        return false;
    }
    for (unsigned int i = 0; i < p1_order; i++)
    {
        if (std::abs(p1.getCoeff(i) - p2.getCoeff(i)) >= GeomConstants::FIXED_EPSILON)
        {
            return false;
        }
    }
    return true;
}

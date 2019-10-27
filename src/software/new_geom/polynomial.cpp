#include "software/new_geom/polynomial.h"

Polynomial::Polynomial() {}

Polynomial::Polynomial(const std::vector<double> &coeffs) : coeffs(coeffs) {}

Polynomial::Polynomial(const std::initializer_list<double> &coeffs) : coeffs(coeffs) {}

Polynomial::Polynomial(const std::pair<double, double> &constraint1,
                       const std::pair<double, double> &constraint2)
{
    if (constraint1.first == constraint2.first)
    {
        throw std::invalid_argument("Both inputs are equal - does not define a function");
    }
    double slope = (constraint2.second - constraint1.second) /
                   (constraint2.first - constraint1.first);
    coeffs.push_back(constraint1.second - (constraint1.first * slope));
    coeffs.push_back(slope);
}

double Polynomial::getCoeff(unsigned int order) const
{
    return order >= coeffs.size() ? 0 : coeffs[order];
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
        for (size_t i = coeffs.size() - 1; i >= 0; i++)
        {
            if (std::abs(coeffs[i]) >= EPSILON)
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
    // If this has a large performance impact, could be improved
    // by using Horner's Method:
    // https://www.geeksforgeeks.org/horners-method-polynomial-evaluation/
    double retval = 0;
    for (unsigned int i = 0; i <= getOrder(); i++)
    {
        retval += getCoeff(i) * std::pow(val, i);
    }
    return retval;
}

Polynomial operator+(const Polynomial &p1, const Polynomial &p2)
{
    Polynomial sum;
    for (unsigned int i = 0; i <= std::max(p1.getOrder(), p2.getOrder()); i++)
    {
        sum.setCoeff(i, p1.getCoeff(i) + p2.getCoeff(i));
    }
    return sum;
}

Polynomial operator-(const Polynomial &p1, const Polynomial &p2)
{
    Polynomial difference;
    for (unsigned int i = 0; i <= std::max(p1.getOrder(), p2.getOrder()); i++)
    {
        difference.setCoeff(i, p1.getCoeff(i) - p2.getCoeff(i));
    }
    return difference;
}

Polynomial operator*(const Polynomial &p1, const Polynomial &p2)
{
    Polynomial product;
    for (unsigned int i = 0; i <= p1.getOrder(); i++)
    {
        for (unsigned int j = 0; j <= p2.getOrder(); j++)
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
    if (p1.getOrder() != p2.getOrder())
    {
        return false;
    }
    for (unsigned int i = 0; i < p1.getOrder(); i++)
    {
        if (std::abs(p1.getCoeff(i) - p2.getCoeff(i)) >= Polynomial::EPSILON)
        {
            return false;
        }
    }
    return true;
}

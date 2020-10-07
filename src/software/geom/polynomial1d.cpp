#include "software/geom/polynomial1d.h"

#include <Eigen/Dense>
#include <Eigen/QR>
#include <list>
#include <stdexcept>

#include "software/geom/geom_constants.h"

Polynomial1d::Polynomial1d() {}

Polynomial1d::Polynomial1d(const std::vector<double> &coeffs) : coeffs(coeffs) {}

Polynomial1d::Polynomial1d(const std::initializer_list<double> &coeffs)
    : coeffs(std::vector<double>(coeffs))
{
}

Polynomial1d::Polynomial1d(const std::vector<Polynomial1d::Constraint> constraints)
{
    // Check that we have at least two constraints
    if (constraints.size() < 2)
    {
        throw std::invalid_argument(
            "Less then two constraints given, so no unique polynomial solution.");
    }

    // Check that all inputs are unique
    for (size_t i = 0; i < constraints.size(); i++)
    {
        for (size_t j = i + 1; j < constraints.size(); j++)
        {
            if (constraints[i].input == constraints[j].input)
            {
                throw std::invalid_argument(
                    "At least two inputs were equal, does not define a valid set of constraints");
            }
        }
    }

    // Solve for the coefficients
    Eigen::MatrixXd A(constraints.size(), constraints.size());
    Eigen::VectorXd b(constraints.size());

    for (size_t row_index = 0; row_index < constraints.size(); row_index++)
    {
        for (size_t col_index = 0; col_index < constraints.size(); col_index++)
        {
            A(row_index, col_index) =
                std::pow(constraints[row_index].input, static_cast<double>(col_index));
            b(row_index) = constraints[row_index].output;
        }
    }

    const Eigen::VectorXd coeff_vector = A.fullPivLu().solve(b);

    for (size_t i = 0; i < constraints.size(); i++)
    {
        coeffs.emplace_back(coeff_vector(i));
    }
}

double Polynomial1d::getCoeff(unsigned int order) const
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

void Polynomial1d::setCoeff(unsigned int order, double coeff)
{
    if (order >= coeffs.size())
    {
        coeffs.resize(order + 1, 0);
    }
    coeffs[order] = coeff;
}

unsigned int Polynomial1d::getOrder() const
{
    if (coeffs.size() != 0)
    {
        for (size_t i = coeffs.size(); i > 0; i--)
        {
            if (std::abs(coeffs[i - 1]) >= FIXED_EPSILON)
            {
                return static_cast<unsigned int>(i) - 1;
            }
        }
    }
    // Zero polynomial treated as an order zero polynomial
    return 0;
}

double Polynomial1d::valueAt(double val) const
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

Polynomial1d operator+(const Polynomial1d &p1, const Polynomial1d &p2)
{
    Polynomial1d sum;
    unsigned int max_order = std::max(p1.getOrder(), p2.getOrder());
    for (unsigned int i = 0; i <= max_order; i++)
    {
        sum.setCoeff(i, p1.getCoeff(i) + p2.getCoeff(i));
    }
    return sum;
}

Polynomial1d operator-(const Polynomial1d &p1, const Polynomial1d &p2)
{
    Polynomial1d difference;
    unsigned int max_order = std::max(p1.getOrder(), p2.getOrder());
    for (unsigned int i = 0; i <= max_order; i++)
    {
        difference.setCoeff(i, p1.getCoeff(i) - p2.getCoeff(i));
    }
    return difference;
}

Polynomial1d operator*(const Polynomial1d &p1, const Polynomial1d &p2)
{
    Polynomial1d product;
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

Polynomial1d &operator+=(Polynomial1d &p1, const Polynomial1d &p2)
{
    return p1 = p1 + p2;
}

Polynomial1d &operator-=(Polynomial1d &p1, const Polynomial1d &p2)
{
    return p1 = p1 - p2;
}

Polynomial1d &operator*=(Polynomial1d &p1, const Polynomial1d &p2)
{
    return p1 = p1 * p2;
}

bool operator==(const Polynomial1d &p1, const Polynomial1d &p2)
{
    unsigned int p1_order = p1.getOrder();
    unsigned int p2_order = p2.getOrder();
    if (p1_order != p2_order)
    {
        return false;
    }
    for (unsigned int i = 0; i < p1_order; i++)
    {
        if (std::abs(p1.getCoeff(i) - p2.getCoeff(i)) >= FIXED_EPSILON)
        {
            return false;
        }
    }
    return true;
}

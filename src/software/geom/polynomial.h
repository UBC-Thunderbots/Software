#pragma once

#include <cmath>
#include <vector>

#include "software/new_geom/point.h"

/**
 * Polynomial is a representation of a polynomial
 * that can calculate values at a given input
 */
class Polynomial
{
   public:
    Polynomial() = delete;
    /**
     * Construct a polynomial from coefficients
     * s.t. n = coeffs.size() == the degree of the polynomial
     * and of the form coeffs[0]*x^(n-1)
     * + coeffs[1]*x^(n-2) + ... + coeffs[n-1]
     *
     * @param coeffs coefficients of the polynomial
     *      * must be
     *
     * @throws std::invalid_argument if coeffs[0] == 0
     */
    explicit Polynomial(const std::vector<double>& coeffs);

    /**
     * Construct a polynomial from coefficients
     * s.t. n = coeffs.size() == the degree of the polynomial
     * and of the form coeffs[0]*x^(n-1)
     * + coeffs[1]*x^(n-2) + ... + coeffs[n-1]
     *
     * @param coeffs coefficients of the polynomial
     *
     * @throws std::invalid_argument if coeffs[0] == 0
     */
    explicit Polynomial(const std::initializer_list<double>& coeffs);

    /**
     * Construct a linear polynomial from two pairs of input/output
     *
     * @param constraint1 first pair of values: first is the input, second is the output
     * @param constraint2 second pair of values: first is the input, second is the output
     *
     * @throws std::invalid_argument if constraint1.first == constraint2.first
     */
    explicit Polynomial(const std::pair<double, double>& constraint1,
                        const std::pair<double, double>& constraint2);

    /**
     * Returns the coefficients of the polynomial
     *
     * @return the coefficients of the polynomial
     */
    const std::vector<double>& getCoeffs() const;

    /**
     * Calculates the value of polynomial evaluated at value val
     *
     * @param val value to evaluate polynomial
     *
     * @return value of polynomial evaluated at value val
     */
    double valueAt(double val) const;

   private:
    // the coefficients for the polynomial
    // of the form coeffs[0]*x^(n-1) + coeffs[1]*x^(n-2)
    // + ... + coeffs[n-1]
    std::vector<double> coeffs;
};

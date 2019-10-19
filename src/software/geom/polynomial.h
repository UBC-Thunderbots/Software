#pragma once

#include <cmath>
#include <vector>

#include "software/geom/point.h"

/**
 * Polynomial is a representation of a polynomial
 * of the form coeffs[0] + coeffs[1] * + coeff[2] * x^2...
 */
class Polynomial
{
   public:
    Polynomial() = delete;
    /**
     * Construct a polynomial from coefficients
     *
     * @param coeffs coefficients of the polynomial
     *
     * @note will prune off coeffs to enforce invariant:
     *  *coeffs[coeffs.size()-1] != 0
     *  *order of polynomial is coeffs.size()
     */
    explicit Polynomial(const std::vector<double>& coeffs);

    /**
     * Construct a polynomial from coefficients
     *
     * @param coeffs coefficients of the polynomial
     *
     * @note will prune off coeffs to enforce invariant:
     *  *coeffs[coeffs.size()-1] != 0
     *  *order of polynomial is coeffs.size()
     */
    explicit Polynomial(const std::initializer_list<double>& coeffs);

    /**
     * Construct a linear polynomial from two pairs of values
     *
     * @param p1 first pair of values: first is the input, second is the output
     * @param p2 second pair of values: first is the input, second is the output
     */
    explicit Polynomial(const std::pair<double, double>& p1,
                        const std::pair<double, double>& p2);

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
    double calculateValue(double val) const;

   private:
    // the coefficients for the polynomial
    std::vector<double> coeffs;

    void pruneCoeffs(void);
};

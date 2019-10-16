#pragma once

#include <cmath>
#include <vector>

#include "software/geom/point.h"

/**
 * Polynomial is a representation of a polynomial
 * of the form coeffs[0] + coeffs[1] * x + coeff[2] * x^2...
 */
class Polynomial
{
   public:
    Polynomial() = delete;
    /**
     * Construct a polynomial from coefficients
     *
     * @param x_coeffs x coefficients of the polynomial
     * @param y_coeffs y coefficients of the polynomial
     *
     * @note will prune off coeffs to enforce invariant:
     *  *coeffs[coeffs.size()-1] != 0
     *  *order of polynomial is coeffs.size()
     */
    explicit Polynomial(const std::vector<double>& x_coeffs, const std::vector<double>& y_coeffs);

    /**
     * Construct a polynomial from coefficients
     *
     * @param x_coeffs x coefficients of the polynomial
     * @param y_coeffs y coefficients of the polynomial
     *
     * @note will prune off coeffs to enforce invariant:
     *  *coeffs[coeffs.size()-1] != 0
     *  *order of polynomial is coeffs.size()
     */
    explicit Polynomial(const std::initializer_list<double>& x_coeffs, const std::initializer_list<double>& y_coeffs);

    /**
     * Returns the x coefficients of the polynomial
     *
     * @return the x coefficients of the polynomial
     */
    const std::vector<double>& getXCoeffs() const;

    /**
     * Returns the y coefficients of the polynomial
     *
     * @return the y coefficients of the polynomial
     */
    const std::vector<double>& getYCoeffs() const;

    /**
     * Calculates the value of polynomial evaluated at time t
     *
     * @param t time to evaluate polynomial
     *
     * @return value of polynomial evaluated at time t
     */
    Point calculateValue(double t) const;

   private:
    // the coefficients for the polynomial
    std::vector<double> x_coeffs;
    std::vector<double> y_coeffs;

    void pruneCoeffs(void);
};

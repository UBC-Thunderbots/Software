#pragma once

#include <cmath>
#include <vector>

/**
 * Polynomial is a representation of a polynomial
 * that can calculate values at a given input
 */
class Polynomial
{
   public:
    /**
     * Construct a zero polynomial
     */
    explicit Polynomial();

    /**
     * Construct a polynomial from coefficients of the form
     * coeffs[0] + coeffs[1]*x^(1) + ... + coeffs[n-1]*x^(n-1)
     *
     * @param coeffs coefficients of the polynomial
     */
    explicit Polynomial(const std::vector<double> &coeffs);

    /**
     * Construct a polynomial from coefficients of the form
     * coeffs[0] + coeffs[1]*x^(1) + ... + coeffs[n-1]*x^(n-1)
     *
     * @param coeffs coefficients of the polynomial
     */
    explicit Polynomial(const std::initializer_list<double> &coeffs);

    /**
     * Returns the coefficient of the term of given order
     *
     * @param order the order of the term
     * @return the coefficient of the term
     */
    double getCoeff(unsigned int order) const;

    /**
     * Sets the coefficient of the term of given order
     *
     * @param order the order of the term to set the coefficient
     * @param coeff the coefficient
     */
    void setCoeff(unsigned int order, double coeff);

    /**
     * Returns the order of the Polynomial
     *
     * @return the index of the highest order non-zero coefficient
     */
    unsigned int getOrder() const;

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
    // of the form coeffs[0] + coeffs[1]*x^(1)
    // + ... + coeff[n-1]*x^(n-1)
    std::vector<double> coeffs;
};

/**
 * Adds two Polynomials
 *
 * @param p1 the first Polynomial
 * @param p2 the second Polynomial
 *
 * @return the sum of the Polynomials
 */
Polynomial operator+(const Polynomial &p1, const Polynomial &p2);

/**
 * Subtracts two Polynomials
 *
 * @param p1 the first Polynomial
 * @param p2 the second Polynomial
 *
 * @return the difference of the Polynomials
 */
Polynomial operator-(const Polynomial &p1, const Polynomial &p2);

/**
 * Multiplies two Polynomials
 *
 * @param p1 the first Polynomial
 * @param p2 the second Polynomial
 *
 * @return the product of the Polynomials
 */
Polynomial operator*(const Polynomial &p1, const Polynomial &p2);

/**
 * Adds a Polynomial to another Polynomial
 *
 * @param p1 the Polynomial to add to.
 * @param p2 the Polynomial to add.
 *
 * @return the new Polynomial
 */
Polynomial &operator+=(Polynomial &p1, const Polynomial &p2);

/**
 * Subtracts a Polynomial from a Polynomial
 *
 * @param p1 the Polynomial to subtract from.
 * @param p2 the Polynomial to subtract.
 *
 * @return the new Polynomial
 */
Polynomial &operator-=(Polynomial &p1, const Polynomial &p2);

/**
 * Multiplies a Polynomial to another Polynomial
 *
 * @param p1 the Polynomial to multiply
 * @param p2 the Polynomial to multiply by
 *
 * @return the new Polynomial
 */
Polynomial &operator*=(Polynomial &p1, const Polynomial &p2);

/**
 * Compares two Polynomials for equality
 *
 * @param p1 the first Polynomial.
 * @param p2 the second Polynomial.
 *
 * @return true if p1 is equal to p2, and false otherwise.
 */
bool operator==(const Polynomial &p1, const Polynomial &p2);

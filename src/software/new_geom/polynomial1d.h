#pragma once

#include <cmath>
#include <vector>

/**
 * Polynomial1d is a representation of a polynomial
 * that can calculate values at a given input
 */
class Polynomial1d
{
   public:
    // A helper struct used to represent constraints
    typedef struct
    {
        double input;
        double output;
    } Constraint;

    /**
     * Construct a zero polynomial
     */
    explicit Polynomial1d();

    /**
     * Construct a polynomial from coefficients of the form
     * coeffs[0] + coeffs[1]*x^(1) + ... + coeffs[n-1]*x^(n-1)
     *
     * @param coeffs coefficients of the polynomial
     */
    explicit Polynomial1d(const std::vector<double> &coeffs);

    /**
     * Construct a polynomial from coefficients of the form
     * coeffs[0] + coeffs[1]*x^(1) + ... + coeffs[n-1]*x^(n-1)
     *
     * @param coeffs coefficients of the polynomial
     */
    explicit Polynomial1d(const std::initializer_list<double> &coeffs);

    /**
     * Construct a polynomial from a series of constraints (input/output pairs)
     *
     * @param constraints A series of pairs, where each pair is of the form
     *                    {input, output}, indicating the output that the created
     *                    polynomial must have for a given input.
     * @param constraints A series of constraints that define the polynomial. Note that
     *                    all inputs must be unique, and there must be at least two
     *                    constraints.
     * @throws std::invalid_argument if any two inputs are equal, or if there are less
     *                               then two constraints
     */
    explicit Polynomial1d(const std::vector<Constraint> constraints);

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
     * Returns the order of the Polynomial1d
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
 * @param p1 the first Polynomial1d
 * @param p2 the second Polynomial1d
 *
 * @return the sum of the Polynomials
 */
Polynomial1d operator+(const Polynomial1d &p1, const Polynomial1d &p2);

/**
 * Subtracts two Polynomials
 *
 * @param p1 the first Polynomial1d
 * @param p2 the second Polynomial1d
 *
 * @return the difference of the Polynomials
 */
Polynomial1d operator-(const Polynomial1d &p1, const Polynomial1d &p2);

/**
 * Multiplies two Polynomials
 *
 * @param p1 the first Polynomial1d
 * @param p2 the second Polynomial1d
 *
 * @return the product of the Polynomials
 */
Polynomial1d operator*(const Polynomial1d &p1, const Polynomial1d &p2);

/**
 * Adds a Polynomial1d to another Polynomial1d
 *
 * @param p1 the Polynomial1d to add to.
 * @param p2 the Polynomial1d to add.
 *
 * @return the new Polynomial1d
 */
Polynomial1d &operator+=(Polynomial1d &p1, const Polynomial1d &p2);

/**
 * Subtracts a Polynomial1d from a Polynomial1d
 *
 * @param p1 the Polynomial1d to subtract from.
 * @param p2 the Polynomial1d to subtract.
 *
 * @return the new Polynomial1d
 */
Polynomial1d &operator-=(Polynomial1d &p1, const Polynomial1d &p2);

/**
 * Multiplies a Polynomial1d to another Polynomial1d
 *
 * @param p1 the Polynomial1d to multiply
 * @param p2 the Polynomial1d to multiply by
 *
 * @return the new Polynomial1d
 */
Polynomial1d &operator*=(Polynomial1d &p1, const Polynomial1d &p2);

/**
 * Compares two Polynomials for equality
 *
 * @param p1 the first Polynomial1d.
 * @param p2 the second Polynomial1d.
 *
 * @return true if p1 is equal to p2, and false otherwise.
 */
bool operator==(const Polynomial1d &p1, const Polynomial1d &p2);

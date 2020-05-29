#pragma once

#include "software/new_geom/point.h"
#include "software/new_geom/polynomial1d.h"

/**
 * A 2D polynomial, represented as two polynomials, x(t) and y(t)
 */
class Polynomial2d
{
   public:
    /**
     * Construct a 2D polynomial. This will construct the polynomial such that
     * it always returns the point (0,0).
     */
    Polynomial2d();

    /**
     * Construct a 2d polynomial from two underlying 1d polynomials
     *
     * Note that these polynomials do not necessarily need to be of the same order.
     *
     * @param poly_x x(t), defines the x-coordinate of this 2D polynomial
     * @param poly_y y(t), defines the y-coordinate of this 2D polynomial
     */
    Polynomial2d(Polynomial1d poly_x, Polynomial1d poly_y);

    /**
     * Calculates the value of polynomial evaluated at value val
     *
     * @param val value to evaluate polynomial
     *
     * @return value of polynomial evaluated at value val
     */
    Point valueAt(double val) const;

    /**
     * Get the polynomial x(t) underlying this 2d Polynomial
     * @return The polynomial x(t) underlying this 2d Polynomial
     */
    Polynomial1d getPolyX() const;

    /**
     * Get the polynomial y(t) underlying this 2d Polynomial
     * @return The polynomial y(t) underlying this 2d Polynomial
     */
    Polynomial1d getPolyY() const;

   private:
    Polynomial1d poly_x;
    Polynomial1d poly_y;
};

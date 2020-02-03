#pragma once

#include "software/new_geom/point.h"
#include "software/new_geom/vector.h"

/**
 * A 2D line.
 */
class Line final
{
   public:
    /**
     * Struct to wrap the coefficients in the form a*x + b*y + c = 0
     */
    struct Coeffs
    {
        double a;
        double b;
        double c;
    };

    Line() = delete;

    /**
     * Creates a Line from two Points
     *
     * @param first the first Point
     * @param second the second Point
     */
    explicit Line(const Point& first, const Point& second);

    /**
     * Returns the coefficient struct
     *
     * @return the coefficient struct
     */
    Coeffs getCoeffs() const;

    /**
     * Returns the normal unit vector of the Line
     *
     * @return the normal unit vector of the Line
     */
    Vector toNormalUnitVector() const;

    /**
     * Reflects the Line about the line y = x
     */
    void swapXY();

    /**
     * Returns the x value corresponding with the given y value
     * @param y
     * @return the x value that corresponds with the given y value
     */
    double x(double y) const;

    /**
     * Returns the y value corresponding with the given x value
     * @param x
     * @return the y value that corresponds with the given x value
     */
    double y(double x) const;

   private:
    // Coefficients for a line
    Coeffs coeffs;
};

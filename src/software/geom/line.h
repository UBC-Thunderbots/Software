#pragma once

#include "software/geom/point.h"
#include "software/geom/vector.h"

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
    const Coeffs& getCoeffs() const;

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

   private:
    // Coefficients for a line
    Coeffs coeffs;
};

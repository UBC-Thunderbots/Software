#pragma once

#include "software/new_geom/point.h"
#include "software/new_geom/vector.h"

/**
 * A 2D line.
 */
class Line final
{
   public:
    Line() = delete;

    /**
     * Creates a Line from two Points
     *
     * @param first the first Point
     * @param second the second Point
     */
    explicit Line(const Point& first, const Point& second);

    struct Coeffs
    {
        double a;
        double b;
        double c;
    };

    /**
     * Returns the coefficient array in the form coeffs[0]*x + coeffs[1]*y + coeffs[2] = 0
     *
     * @return the coefficient array
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

   private:
    // Coefficients for a line in the form coeffs[0]*x + coeffs[1]*y + coeffs[2] = 0
    std::array<double, 3> coeffs;
};

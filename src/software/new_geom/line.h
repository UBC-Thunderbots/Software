#pragma once

#include <stdexcept>

#include "software/new_geom/polynomial.h"

class Line final : public Polynomial
{
   public:
    /**
     * Creates a line with zero y-intercept and zero slope
     */
    inline explicit Line() {}

    /**
     * Creates a line from a y-intercept and a slope
     *
     * @param y_intercept the y_intercept of the line
     * @param slope the slope of the line
     */
    inline explicit Line(double y_intercept, double slope)
        : Polynomial({y_intercept, slope})
    {
    }

    /**
     * Sets the coefficient of the term of the given order
     *
     * @param order the order of the term to set the coefficient
     * @param coeff the coefficient
     *
     * @throws std::invalid_argument if order > 1
     */
    inline void setCoeff(unsigned int order, double coeff) override
    {
        if (order > 1)
        {
            throw std::invalid_argument(
                "Tried to set the coefficient of term with order greater than 1");
        }
        Polynomial::setCoeff(order, coeff);
    };
};

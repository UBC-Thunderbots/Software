#pragma once

#include <stdexcept>

#include "software/new_geom/polynomial.h"

class Line final : public Polynomial
{
   public:
    /**
     * Creates a line with zero y-intercept and zero slope
     */
    Line();

    /**
     * Creates a line from a y-intercept and a slope
     *
     * @param y_intercept the y_intercept of the line
     * @param slope the slope of the line
     */
    explicit Line(double y_intercept, double slope);

    /**
     * Sets the coefficient of the term of the given order
     *
     * @param order the order of the term to set the coefficient
     * @param coeff the coefficient
     *
     * @throws std::invalid_argument if order > 1
     */
    void setCoeff(unsigned int order, double coeff) override;
};

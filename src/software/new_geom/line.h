#pragma once

#include <stdexcept>

#include "software/new_geom/polynomial.h"

/**
 * A 2D line.
 */
class Line final
{
   public:
    /**
     * Creates a line with zero y-intercept and zero slope
     */
    Line();

    /**
     * Creates a line from a y-intercept and a slope
     *
     * @param y_intercept the y-intercept
     * @param slope the slope
     */
    explicit Line(double y_intercept, double slope);

    /**
     * Returns the y-intercept
     *
     * @return the y-intercept
     */
    double getYIntercept();

    /**
     * Returns the slope
     *
     * @return the slope
     */
    double getSlope();

    /**
     * Sets the y-intercept
     *
     * @param y_intercept the new y-intercept
     */
    void setYIntercept(double y_intercept);

    /**
     * Sets the slope
     *
     * @param slope the new slope
     */
    void setSlope(double slope);

    /**
     * Calculates the value of line evaluated at value val
     *
     * @param val value to evaluate line
     *
     * @return value of line evaluated at value val
     */
    double valueAt(double val);

   private:
    /**
     * Polynomial object that represents the line
     */
    Polynomial line;
};

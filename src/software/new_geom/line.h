#pragma once

#include "software/new_geom/polynomial.h"
#include "software/new_geom/point.h"

/**
 * A 2D line.
 */
class Line final
{
   public:
    /**
     * Creates a degenerate line with zero Points and zero Polynomials
     */
    Line();

    /**
     * Creates a line from two points
     *
     * @param first the first point
     * @param second the second point
     */
    explicit Line(const Point& first, const Point& second);

    /**
     * Returns the first point
     *
     * @return the first point
     */
    Point getFirst() const;

    /**
     * Returns the second point
     *
     * @return the second point
     */
    Point getSecond() const;

    /**
     * Returns the slope
     *
     * @return the slope
     */
    double getSlope() const;

    /**
     * Sets the first point
     *
     * @param first the new first point
     */
    void setFirst(const Point& first);

    /**
     * Sets the second point
     *
     * @param second the new second point
     */
    void setSecond(const Point& second);

    /**
     * Calculates the Point evaluated at val, starting from the first point and moving val units along the line towards the second point
     *
     * @param val value to evaluate line
     *
     * @return Point on line evaluated at val
     */
    Point valueAt(double val) const;

   private:
    /**
     * The first point that defines the starting point of the line
     */
    Point first;

    /**
     * The second point that defines the direction of the line
     */
    Point second;

    /**
     * Parametric equation for the x-axis
     */
    Polynomial x;

    /**
     * Parametric equation for the y-axis
     */
    Polynomial y;

    void calculateLine(const Point& first, const Point& second);
};

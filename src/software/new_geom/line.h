#pragma once

#include "software/new_geom/point.h"
#include "software/new_geom/polynomial.h"

/**
 * A 2D line.
 */
class Line final
{
   public:
    /**
     * Creates a degenerate Line with zero Points and zero Polynomials
     */
    Line();

    /**
     * Creates a Line from two Points
     *
     * @param first the first Point
     * @param second the second Point
     */
    explicit Line(const Point& first, const Point& second);

    /**
     * Returns the first Point
     *
     * @return the first Point
     */
    Point getFirst() const;

    /**
     * Returns the second Point
     *
     * @return the second Point
     */
    Point getSecond() const;

    /**
     * Returns the slope
     *
     * @return the slope
     */
    double getSlope() const;

    /**
     * Sets the first Point
     *
     * @param first the new first Point
     */
    void setFirst(const Point& first);

    /**
     * Sets the second Point
     *
     * @param second the new second Point
     */
    void setSecond(const Point& second);

    /**
     * Calculates the Point evaluated at val, starting from the first Point and moving val
     * units along the Line towards the second Point
     *
     * @param val the value to evaluate Line
     *
     * @return Point on Line evaluated at val
     */
    Point valueAt(double val) const;

   private:
    /**
     * The first point that defines the starting point of the Line
     */
    Point first;

    /**
     * The second point that defines the direction of the Line
     */
    Point second;

    /**
     * Parametric equation for the x component of Line
     */
    Polynomial x;

    /**
     * Parametric equation for the y component of Line
     */
    Polynomial y;

    /**
     * Calculates the parametric equations based on Points given and sets x and y
     * accordingly
     *
     * @param first the first Point
     * @param second the second Point
     */
    void calculateLine(const Point& first, const Point& second);
};

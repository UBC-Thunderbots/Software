#pragma once

#include "software/geom/point.h"

class Ray final
{
   public:
    /**
     * Creates a Ray starting at (0, 0) pointing along the positive x-axis
     */
    explicit Ray();

    /**
     * Creates a Ray with a start point and a direction angle
     *
     * @param start the start point of Ray
     * @param direction the direction angle of Ray
     */
    explicit Ray(const Point& start, const Angle& direction);

    /**
     * Creates a Ray with a start point and a direction vector
     *
     * @param start the start point of Ray
     * @param direction the direction vector of Ray
     */
    explicit Ray(const Point& start, const Vector& direction);

    /**
     * Returns the start point of this Ray
     *
     * @return the start point of this Ray
     */
    const Point& getStart() const;

    /**
     * Returns the direction of this Ray
     *
     * @return the direction of this Ray
     */
    const Angle& getDirection() const;

    /**
     * Sets the start point of this Ray
     *
     * @param point the new start point
     */
    void setStart(const Point& start);

    /**
     * Sets the direction of this Ray with an angle
     *
     * @param direction the new direction as an angle
     */
    void setDirection(const Angle& direction);

    /**
     * Sets the direction of this Ray with a vector
     *
     * @param direction the new direction as a vector
     */
    void setDirection(const Vector& direction);

    /**
     * Rotates this Ray counterclockwise by an angle
     *
     * @param angle the angle to rotate the Ray
     */
    void rotate(const Angle& angle);

    /**
     * Returns a unit-magnitude Vector in the direction of the Ray
     *
     * @return a unit-magnitude Vector in the direction of the Ray
     */
    Vector toUnitVector() const;

   private:
    /**
     * The start point of the Ray.
     */
    Point start_;

    /**
     * The direction of the Ray.
     */
    Angle direction_;
};

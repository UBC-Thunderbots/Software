#pragma once

#include "software/new_geom/point.h"

class Ray final
{
   public:
    /**
     * Creates a degenerate Ray at (0, 0)
     */
    inline explicit Ray() {}

    /**
     * Creates a Ray with a start point and a direction angle
     *
     * @param start the start point of Ray
     * @param direction the direction angle of Ray
     */
    inline explicit Ray(const Point& start, const Angle& direction)
        : start_(start), direction_(direction)
    {
    }

    /**
     * Creates a Ray with a start point and a direction vector
     *
     * @param start the start point of Ray
     * @param direction the direction vector of Ray
     */
    inline explicit Ray(const Point& start, const Vector& direction)
        : Ray(start, direction.orientation())
    {
    }

    /**
     * Returns the start point of this Ray
     *
     * @return the start point of this Ray
     */
    inline Point getStart() const
    {
        return start_;
    }

    /**
     * Returns the direction of this Ray
     *
     * @return the direction of this Ray
     */
    inline Angle getDirection() const
    {
        return direction_;
    }

    /**
     * Sets the start point of this Ray
     *
     * @param point the new start point
     */
    inline void setStart(const Point& start)
    {
        start_ = start;
    }

    /**
     * Sets the direction of this Ray with an angle
     *
     * @param direction the new direction as an angle
     */
    inline void setDirection(const Angle& direction)
    {
        direction_ = direction;
    }

    /**
     * Sets the direction of this Ray with a vector
     *
     * @param direction the new direction as a vector
     */
    inline void setDirection(const Vector& direction)
    {
        direction_ = direction.orientation();
    }

    /**
     * Rotates this Ray counterclockwise by an angle
     *
     * @param angle the angle to rotate the Ray
     */
    inline void rotate(Angle angle)
    {
        direction_ += angle;
    }

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

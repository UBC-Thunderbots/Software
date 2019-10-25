#pragma once

#include "software/geom/line.h"
#include "software/geom/point.h"

class Ray final
{
   public:
    /**
     * Creates a degenerate Ray at (0, 0)
     */
    inline explicit constexpr Ray() {}

    /**
     * Creates a Ray with a start point and a direction
     *
     * @param start the starting point of Ray
     * @param direction the direction of Ray
     */
    inline explicit Ray(const Point& start, const Vector& direction)
        : start_(start), direction_(direction)
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
    inline Vector getDirection() const
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
     * Sets the direction of this Ray
     *
     * @param direction the new direction
     */
    inline void setDirection(const Vector& direction)
    {
        direction_ = direction;
    }

    /**
     * Rotates this Ray counterclockwise by an angle
     *
     * @param angle the angle to rotate the Ray
     */
    inline void rotate(Angle angle)
    {
        direction_ = direction_.rotate(angle);
    }

   private:
    /**
     * The start point of the ray.
     */
    Point start_;

    /**
     * The direction of the ray.
     */
    Vector direction_;
};

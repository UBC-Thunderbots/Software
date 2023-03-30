#pragma once

#include "software/geom/point.h"
/**
 * A path point in an agent's path
 */
class PathPoint
{
   public:
    /**
     * @param position The position of this path point
     * @param speed The speed at this path point
     */
    explicit PathPoint(const Point &position, double speed);

    /**
     * Gets the position of the path point
     * @return Vector position
     */
    Point getPosition() const;

    /**
     * Gets the speed of the path point
     * @return float speed
     */
    double getSpeed() const;

   private:
    // position for the path point
    Point position;
    // desired speed for the path point
    double speed;
};

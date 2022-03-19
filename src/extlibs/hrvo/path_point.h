#pragma once
#include <vector>

#include "software/geom/vector.h"
/**
 * A path point in an agent's path
 */
class PathPoint
{
   public:
    /**
     * @param position  The position of this path point
     */
    explicit PathPoint(const Vector &position);

    /**
     * @param position The position of this path point
     * @param speed_at_position The speed at this path point
     */
    explicit PathPoint(const Vector &position, const float speed_at_position);

    Vector getPosition() const;

    float getSpeed() const;

   private:
    // position for the path point
    Vector position_;
    // desired speed for the path point
    float speed_at_destination;

};

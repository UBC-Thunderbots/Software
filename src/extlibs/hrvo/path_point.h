#pragma once
#include <vector>

#include "extlibs/hrvo/vector2.h"

class Simulator;

/**
 * A path point in an agent's path
 */
class PathPoint
{
   public:
    /**
     * @param position  The position of this path point
     */
    explicit PathPoint(const Vector2 &position);

    /**
     * @param position The position of this path point
     * @param speed_at_position The speed at this path point
     */
    explicit PathPoint(const Vector2 &position, const float speed_at_position);

   private:
    // position for the path point
    Vector2 position_;
    // desired speed for the path point
    float speed_at_destination;

    friend class Path;
};

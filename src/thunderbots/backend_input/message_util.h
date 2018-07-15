#ifndef THUNDERBOTS_VISION_UTIL_H
#define THUNDERBOTS_VISION_UTIL_H

#include "geom/point.h"
#include "thunderbots_msgs/Ball.h"

class VisionUtil
{
   public:
    /**
     * Creates a Ball msg given the position and velocity of a ball
     *
     * @param position the position of the ball
     * @param velocity the velocity of the ball
     *
     * @return a Ball message containing the ball information
     */
    static thunderbots_msgs::Ball createBallMsg(
        const Point &position, const Point &velocity);
};

#endif  // THUNDERBOTS_VISION_UTIL_H

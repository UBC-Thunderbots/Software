#pragma once

#include "ai/world/ball.h"
#include "thunderbots_msgs/Ball.h"

namespace Util
{
    /**
     * Functions in this namespace provide functions that assist in converting between
     * a ROS message and its equivalent class
     */
    namespace ROSMessages
    {
        /**
         * Given a Ball message, constructs and returns a Ball object
         *
         * @param ball_msg The message containing the ball message data
         * @return A Ball object created with the given ball message data
         */
        Ball createBallFromROSMessage(const thunderbots_msgs::Ball& ball_msg);
    }
}

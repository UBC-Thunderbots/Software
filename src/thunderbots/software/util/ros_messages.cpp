#include "util/ros_messages.h"

namespace Util
{
    namespace ROSMessages
    {
        Ball createBallFromROSMessage(const thunderbots_msgs::Ball& ball_msg)
        {
            Point ball_position  = Point(ball_msg.position.x, ball_msg.position.y);
            Vector ball_velocity = Vector(ball_msg.velocity.x, ball_msg.velocity.y);

            Ball ball = Ball(ball_position, ball_velocity);

            return ball;
        }
    }
}
#include "message_util.h"

thunderbots_msgs::Ball VisionUtil::createBallMsg(
    const Point &position, const Point &velocity)
{
    thunderbots_msgs::Ball ball_msg;

    ball_msg.position.x = position.x();
    ball_msg.position.y = position.y();

    ball_msg.velocity.x = velocity.x();
    ball_msg.velocity.y = velocity.y();

    return ball_msg;
}

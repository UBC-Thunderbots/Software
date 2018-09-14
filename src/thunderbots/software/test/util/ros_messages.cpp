#include "util/ros_messages.h"
#include <gtest/gtest.h>

TEST(ROSMessageUtilTest, create_ball_from_ros_message)
{
    thunderbots_msgs::Ball ball_msg;

    ball_msg.position.x = 1.2;
    ball_msg.position.y = -8.07;
    ball_msg.velocity.x = 0;
    ball_msg.velocity.y = 3;

    Ball ball = Util::ROSMessages::createBallFromROSMessage(ball_msg);

    EXPECT_EQ(Ball(Point(1.2, -8.07), Vector(0, 3)), ball);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

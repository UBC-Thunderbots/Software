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

TEST(ROSMessageUtilTest, create_robot_from_ros_message)
{
    unsigned int id                  = 2;
    Point position                   = Point(1.2, -8.07);
    Vector velocity                  = Vector(0, 3);
    Angle orientation                = Angle::ofRadians(1.16);
    AngularVelocity angular_velocity = AngularVelocity::ofRadians(-0.85);

    thunderbots_msgs::Robot robot_msg;

    robot_msg.id               = id;
    robot_msg.position.x       = position.x();
    robot_msg.position.y       = position.y();
    robot_msg.velocity.x       = velocity.x();
    robot_msg.velocity.y       = velocity.y();
    robot_msg.orientation      = orientation.toRadians();
    robot_msg.angular_velocity = angular_velocity.toRadians();

    Robot robot = Util::ROSMessages::createRobotFromROSMessage(robot_msg);

    EXPECT_EQ(Robot(id, position, velocity, orientation, angular_velocity), robot);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

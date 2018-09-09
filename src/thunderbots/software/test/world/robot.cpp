#include "ai/world/robot.h"
#include <gtest/gtest.h>

TEST(RobotTest, construction)
{
    Robot robot = Robot(0);

    EXPECT_EQ(Point(), robot.position());
    EXPECT_EQ(Vector(), robot.velocity());
    EXPECT_EQ(Angle::zero(), robot.orientation());
    EXPECT_EQ(AngularVelocity::zero(), robot.angularVelocity());
}

TEST(RobotTest, update_and_accessors)
{
    Robot robot = Robot(0);

    robot.update(Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
                 AngularVelocity::ofRadians(1.1));

    EXPECT_EQ(Point(-1.2, 3), robot.position());
    EXPECT_EQ(Vector(2.2, -0.05), robot.velocity());
    EXPECT_EQ(Angle::quarter(), robot.orientation());
    EXPECT_EQ(AngularVelocity::ofRadians(1.1), robot.angularVelocity());
}

TEST(RobotTest, equality_operators)
{
    Robot robot_0_0 = Robot(0);
    robot_0_0.update(Point(1, -1.5), Vector(-0.7, -0.55), Angle::ofDegrees(100),
                     AngularVelocity::ofDegrees(30));

    Robot robot_0_1 = Robot(0);
    robot_0_1.update(Point(1, -1.5), Vector(-0.7, -0.55), Angle::ofDegrees(100),
                     AngularVelocity::ofDegrees(-30));

    Robot robot_1 = Robot(1);
    robot_1.update(Point(1, -1.5), Vector(-0.7, -0.55), Angle::ofDegrees(100),
                   AngularVelocity::ofDegrees(30));

    Robot robot_2 = Robot(2);
    robot_2.update(Point(3, 1.2), Vector(3, 1), Angle::ofDegrees(0),
                   AngularVelocity::ofDegrees(25));

    EXPECT_EQ(robot_0_0, robot_0_0);
    EXPECT_NE(robot_0_0, robot_0_1);
    EXPECT_NE(robot_0_0, robot_1);
    EXPECT_NE(robot_0_0, robot_2);

    EXPECT_EQ(robot_1, robot_1);
    EXPECT_NE(robot_1, robot_2);

    EXPECT_NE(robot_0_1, robot_1);
    EXPECT_NE(robot_0_1, robot_2);

    // Update robot_2 to be the same as robot_1 (except for the robot id)
    robot_2.update(Point(1, -1.5), Vector(-0.7, -0.55), Angle::ofDegrees(100),
                   AngularVelocity::ofDegrees(30));

    EXPECT_NE(robot_1, robot_2);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#include "ai/world/robot.h"
#include <gtest/gtest.h>

TEST(RobotTest, construct_with_id_only)
{
    Robot robot = Robot(0);

    EXPECT_EQ(0, robot.id());
    EXPECT_EQ(Point(), robot.position());
    EXPECT_EQ(Vector(), robot.velocity());
    EXPECT_EQ(Angle::zero(), robot.orientation());
    EXPECT_EQ(AngularVelocity::zero(), robot.angularVelocity());
}

TEST(RobotTest, construct_with_all_params)
{
    Robot robot = Robot(3, Point(1, 1), Vector(-0.3, 0), Angle::ofRadians(2.2),
                        AngularVelocity::ofRadians(-0.6));

    EXPECT_EQ(3, robot.id());
    EXPECT_EQ(Point(1, 1), robot.position());
    EXPECT_EQ(Vector(-0.3, 0), robot.velocity());
    EXPECT_EQ(Angle::ofRadians(2.2), robot.orientation());
    EXPECT_EQ(AngularVelocity::ofRadians(-0.6), robot.angularVelocity());
}

TEST(RobotTest, update_with_all_params)
{
    Robot robot = Robot(0);

    robot.update(Point(-1.2, 3), Vector(2.2, -0.05), Angle::quarter(),
                 AngularVelocity::ofRadians(1.1));

    EXPECT_EQ(0, robot.id());
    EXPECT_EQ(Point(-1.2, 3), robot.position());
    EXPECT_EQ(Vector(2.2, -0.05), robot.velocity());
    EXPECT_EQ(Angle::quarter(), robot.orientation());
    EXPECT_EQ(AngularVelocity::ofRadians(1.1), robot.angularVelocity());
}

TEST(RobotTest, update_specific_params)
{
    Robot robot = Robot(0);

    // Only update certain parameters and leave the rest as their previous values
    robot.update(robot.position(), Vector(2.2, -0.05), robot.orientation(),
                 AngularVelocity::ofRadians(1.1));

    EXPECT_EQ(0, robot.id());
    EXPECT_EQ(Point(), robot.position());
    EXPECT_EQ(Vector(2.2, -0.05), robot.velocity());
    EXPECT_EQ(Angle::zero(), robot.orientation());
    EXPECT_EQ(AngularVelocity::ofRadians(1.1), robot.angularVelocity());

    // Repeat with a different combination of parameters
    robot.update(Point(-1.2, 3), robot.velocity(), Angle::quarter(),
                 robot.angularVelocity());

    EXPECT_EQ(0, robot.id());
    EXPECT_EQ(Point(-1.2, 3), robot.position());
    EXPECT_EQ(Vector(2.2, -0.05), robot.velocity());
    EXPECT_EQ(Angle::quarter(), robot.orientation());
    EXPECT_EQ(AngularVelocity::ofRadians(1.1), robot.angularVelocity());
}

TEST(RobotTest, update_with_new_robot)
{
    Robot robot = Robot(0);

    Robot update_robot = Robot(0, Point(-1.2, 3), robot.velocity(), Angle::quarter(),
                               robot.angularVelocity());

    robot.update(update_robot);

    EXPECT_EQ(robot, update_robot);

    // TODO: Update this test to also test for a thrown exception once
    // https://github.com/UBC-Thunderbots/Software/issues/16
    // is completed. If a robot is updated using a robot with a different id, an
    // exception should be thrown
}

TEST(RobotTest, get_position_at_current_time)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::ofRadians(0.7));

    EXPECT_EQ(Point(-1.2, 3), robot.position());
}

TEST(RobotTest, get_position_at_future_time)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::ofRadians(0.7));

    EXPECT_EQ(Point(-1.4, 4.04), robot.estimatePositionAtFutureTime(0.4));
    EXPECT_EQ(Point(-1.7, 5.6), robot.estimatePositionAtFutureTime(1));
    EXPECT_EQ(Point(-2.7, 10.8), robot.estimatePositionAtFutureTime(3));

    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::ofRadians(-0.3),
                              AngularVelocity::ofRadians(2));

    EXPECT_EQ(Point(2.4, -1.6), robot_other.estimatePositionAtFutureTime(0.4));
    EXPECT_EQ(Point(4.5, -1), robot_other.estimatePositionAtFutureTime(1));
    EXPECT_EQ(Point(11.5, 1), robot_other.estimatePositionAtFutureTime(3));
}

TEST(RobotTest, get_velocity_at_current_time)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::ofRadians(0.7));

    EXPECT_EQ(Vector(-0.5, 2.6), robot.velocity());
}

TEST(RobotTest, get_velocity_at_future_time)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::ofRadians(0.7));

    EXPECT_EQ(Point(-0.5, 2.6), robot.estimateVelocityAtFutureTime(0.4));
    EXPECT_EQ(Point(-0.5, 2.6), robot.estimateVelocityAtFutureTime(1));
    EXPECT_EQ(Point(-0.5, 2.6), robot.estimateVelocityAtFutureTime(3));

    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::ofRadians(-0.3),
                              AngularVelocity::ofRadians(2));

    EXPECT_EQ(Point(3.5, 1), robot_other.estimateVelocityAtFutureTime(0.4));
    EXPECT_EQ(Point(3.5, 1), robot_other.estimateVelocityAtFutureTime(1));
    EXPECT_EQ(Point(3.5, 1), robot_other.estimateVelocityAtFutureTime(3));
}

TEST(RobotTest, get_orientation_at_current_time)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::ofRadians(0.7));

    EXPECT_EQ(Angle::quarter(), robot.orientation());
}

TEST(RobotTest, get_orientation_at_future_time)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::ofRadians(0.7));

    EXPECT_EQ(Angle::quarter() + Angle::ofRadians(0.28),
              robot.estimateOrientationAtFutureTime(0.4));
    EXPECT_EQ(Angle::quarter() + Angle::ofRadians(0.7),
              robot.estimateOrientationAtFutureTime(1));
    EXPECT_EQ(Angle::quarter() + Angle::ofRadians(2.1),
              robot.estimateOrientationAtFutureTime(3));

    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::ofRadians(-0.3),
                              AngularVelocity::ofRadians(2));

    EXPECT_EQ(Angle::ofRadians(-0.3) + Angle::ofRadians(0.8),
              robot_other.estimateOrientationAtFutureTime(0.4));
    EXPECT_EQ(Angle::ofRadians(-0.3) + Angle::ofRadians(2),
              robot_other.estimateOrientationAtFutureTime(1));
    EXPECT_EQ(Angle::ofRadians(-0.3) + Angle::ofRadians(6),
              robot_other.estimateOrientationAtFutureTime(3));
}

TEST(RobotTest, get_angular_velocity_at_current_time)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::ofRadians(0.7));

    EXPECT_EQ(AngularVelocity::ofRadians(0.7), robot.angularVelocity());
}

TEST(RobotTest, get_angularVelocity_at_future_time)
{
    Robot robot = Robot(0, Point(-1.2, 3), Vector(-0.5, 2.6), Angle::quarter(),
                        AngularVelocity::ofRadians(0.7));

    EXPECT_EQ(AngularVelocity::ofRadians(0.7),
              robot.estimateAngularVelocityAtFutureTime(0.4));
    EXPECT_EQ(AngularVelocity::ofRadians(0.7),
              robot.estimateAngularVelocityAtFutureTime(1));
    EXPECT_EQ(AngularVelocity::ofRadians(0.7),
              robot.estimateAngularVelocityAtFutureTime(3));

    Robot robot_other = Robot(1, Point(1, -2), Vector(3.5, 1), Angle::ofRadians(-0.3),
                              AngularVelocity::ofRadians(2));

    EXPECT_EQ(AngularVelocity::ofRadians(2),
              robot_other.estimateAngularVelocityAtFutureTime(0.4));
    EXPECT_EQ(AngularVelocity::ofRadians(2),
              robot_other.estimateAngularVelocityAtFutureTime(1));
    EXPECT_EQ(AngularVelocity::ofRadians(2),
              robot_other.estimateAngularVelocityAtFutureTime(3));
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

    // Update robot_0_1 to be the same as robot_0
    robot_0_1.update(Point(1, -1.5), Vector(-0.7, -0.55), Angle::ofDegrees(100),
                     AngularVelocity::ofDegrees(30));

    EXPECT_EQ(robot_0_0, robot_0_1);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

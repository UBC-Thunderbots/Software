#include "software/ai/hl/stp/tactic/transition_conditions.h"

#include <gtest/gtest.h>

TEST(TransitionConditionTest, test_move_robot_done)
{
    Robot robot = Robot(0, Point(2, 3), Vector(), Angle::quarter(),
                        AngularVelocity::zero(), Timestamp::fromSeconds(0));
    EXPECT_TRUE(robotReachedDestination(robot, Point(2, 3), Angle::quarter()));
    EXPECT_FALSE(robotReachedDestination(robot, Point(2, 2), Angle::quarter()));
    EXPECT_FALSE(robotReachedDestination(robot, Point(2, 3), Angle::threeQuarter()));
}

TEST(TransitionConditionTest, test_robot_stopped)
{
    Robot stopped_robot = Robot(0, Point(2, 3), Vector(), Angle::quarter(),
                                AngularVelocity::zero(), Timestamp::fromSeconds(0));
    EXPECT_TRUE(robotStopped(stopped_robot));
    Robot moving_robot = Robot(0, Point(2, 3), Vector(3, 1), Angle::quarter(),
                               AngularVelocity::zero(), Timestamp::fromSeconds(0));
    EXPECT_FALSE(robotStopped(moving_robot));
}

TEST(TransitionConditionTest, test_compare_angles)
{
    Angle angle1 = Angle::fromDegrees(130);
    Angle angle2 = Angle::fromDegrees(135);
    Angle angle3 = Angle::fromDegrees(138);
    EXPECT_TRUE(compareAngles(angle1, angle2, Angle::fromDegrees(5)));
    EXPECT_FALSE(compareAngles(angle1, angle3, Angle::fromDegrees(5)));
}

TEST(TransitionConditionTest, test_compare_points)
{
    Point point1 = Point(1, 1);
    Point point2 = Point(1, 2);
    Point point3 = Point(2, 1);
    EXPECT_TRUE(comparePoints(point1, point2, 1.0));
    EXPECT_FALSE(comparePoints(point2, point3, 1.0));
}

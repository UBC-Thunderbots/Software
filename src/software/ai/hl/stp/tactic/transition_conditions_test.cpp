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

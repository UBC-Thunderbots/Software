#include "software/world/robot_state.h"

#include <gtest/gtest.h>

TEST(RobotStateTest, get_position)
{
    RobotState state(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                     AngularVelocity::half());
    EXPECT_EQ(state.position(), Point(1.1, -0.5));
}

TEST(RobotStateTest, get_velocity)
{
    RobotState state(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                     AngularVelocity::half());
    EXPECT_EQ(state.velocity(), Vector(3, 0));
}

TEST(RobotStateTest, get_orientation)
{
    RobotState state(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                     AngularVelocity::half());
    EXPECT_EQ(state.orientation(), Angle::quarter());
}

TEST(RobotStateTest, get_angular_velocity)
{
    RobotState state(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                     AngularVelocity::half());
    EXPECT_EQ(state.angularVelocity(), AngularVelocity::half());
}

TEST(RobotStateTest, compare_identical_states)
{
    RobotState state_1(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                       AngularVelocity::half());
    RobotState state_2(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                       AngularVelocity::half());
    EXPECT_TRUE(state_1 == state_2);
    EXPECT_FALSE(state_1 != state_2);
}

TEST(RobotStateTest, compare_states_with_different_position)
{
    RobotState state_1(Point(1.1, -0.6), Vector(3, 0), Angle::quarter(),
                       AngularVelocity::half());
    RobotState state_2(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                       AngularVelocity::half());
    EXPECT_FALSE(state_1 == state_2);
    EXPECT_TRUE(state_1 != state_2);
}

TEST(RobotStateTest, compare_states_with_different_velocity)
{
    RobotState state_1(Point(1.1, -0.5), Vector(3, 1.2), Angle::quarter(),
                       AngularVelocity::half());
    RobotState state_2(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                       AngularVelocity::half());
    EXPECT_FALSE(state_1 == state_2);
    EXPECT_TRUE(state_1 != state_2);
}

TEST(RobotStateTest, compare_states_with_different_orientation)
{
    RobotState state_1(Point(1.1, -0.5), Vector(3, 0), Angle::half(),
                       AngularVelocity::half());
    RobotState state_2(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                       AngularVelocity::half());
    EXPECT_FALSE(state_1 == state_2);
    EXPECT_TRUE(state_1 != state_2);
}

TEST(RobotStateTest, compare_states_with_different_angular_velocity)
{
    RobotState state_1(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                       AngularVelocity::zero());
    RobotState state_2(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                       AngularVelocity::half());
    EXPECT_FALSE(state_1 == state_2);
    EXPECT_TRUE(state_1 != state_2);
}

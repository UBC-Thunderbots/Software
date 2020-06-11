#include "software/world/robot_state.h"

#include <gtest/gtest.h>

TEST(RobotStateTest, get_position)
{
    RobotState state(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                     AngularVelocity::half());
    EXPECT_EQ(Point(1.1, -0.5), state.position());
}

TEST(RobotStateTest, get_velocity)
{
    RobotState state(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                     AngularVelocity::half());
    EXPECT_EQ(Vector(3, 0), state.velocity());
}

TEST(RobotStateTest, get_orientation)
{
    RobotState state(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                     AngularVelocity::half());
    EXPECT_EQ(Angle::quarter(), state.orientation());
}

TEST(RobotStateTest, get_angular_velocity)
{
    RobotState state(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                     AngularVelocity::half());
    EXPECT_EQ(AngularVelocity::half(), state.angularVelocity());
}

TEST(RobotStateTest, get_default_constructed_height)
{
    RobotState state(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                     AngularVelocity::half());
    EXPECT_EQ(ROBOT_MAX_HEIGHT_METERS, state.height());
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

TEST(RobotStateTest, compare_states_with_different_height)
{
    RobotState state_1(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                       AngularVelocity::zero(), 0.15);
    RobotState state_2(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                       AngularVelocity::zero(), 0.2);
    EXPECT_FALSE(state_1 == state_2);
    EXPECT_TRUE(state_1 != state_2);
}

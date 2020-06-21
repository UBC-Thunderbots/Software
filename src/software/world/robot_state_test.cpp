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

TEST(RobotStateTest, compare_states_with_different_ball_in_beam)
{
    RobotState state_1(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                       AngularVelocity::zero());
    RobotState state_2(state_1);
    EXPECT_FALSE(state_1.ballInMouth());
    EXPECT_FALSE(state_2.ballInMouth());
    state_2.setBallInMouth(true);
    EXPECT_FALSE(state_1 == state_2);
    EXPECT_TRUE(state_1 != state_2);
    EXPECT_FALSE(state_1.ballInMouth());
    EXPECT_TRUE(state_2.ballInMouth());
}

TEST(RobotStateTest, compare_states_with_same_ball_in_beam)
{
    RobotState state_1(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                       AngularVelocity::zero());
    RobotState state_2(state_1);
    EXPECT_FALSE(state_1.ballInMouth());
    EXPECT_FALSE(state_2.ballInMouth());
    EXPECT_TRUE(state_1 == state_2);
    EXPECT_FALSE(state_1 != state_2);
    state_1.setBallInMouth(true);
    state_2.setBallInMouth(true);
    EXPECT_TRUE(state_1 == state_2);
    EXPECT_FALSE(state_1 != state_2);
    EXPECT_TRUE(state_1.ballInMouth());
    EXPECT_TRUE(state_2.ballInMouth());
}

TEST(RobotStateTest, compare_states_with_same_time_since)
{
    RobotState state_1(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                       AngularVelocity::zero());
    RobotState state_2(state_1);
    EXPECT_EQ(std::nullopt, state_1.timeSinceLastChip());
    EXPECT_EQ(std::nullopt, state_1.timeSinceLastKick());
    EXPECT_EQ(std::nullopt, state_2.timeSinceLastChip());
    EXPECT_EQ(std::nullopt, state_2.timeSinceLastKick());
    EXPECT_NE(Duration::fromMilliseconds(5), state_2.timeSinceLastKick());
    EXPECT_TRUE(state_1 == state_2);
    EXPECT_FALSE(state_1 != state_2);
    state_1.setTimeSinceLastChip(Duration::fromMilliseconds(5));
    state_2.setTimeSinceLastChip(Duration::fromMilliseconds(5));
    EXPECT_EQ(state_1.timeSinceLastChip(), Duration::fromMilliseconds(5));
    EXPECT_EQ(state_2.timeSinceLastChip(), Duration::fromMilliseconds(5));
    EXPECT_TRUE(state_1 == state_2);
    EXPECT_FALSE(state_1 != state_2);
    state_1.setTimeSinceLastKick(Duration::fromMilliseconds(5));
    state_2.setTimeSinceLastKick(Duration::fromMilliseconds(5));
    EXPECT_EQ(state_1.timeSinceLastKick(), Duration::fromMilliseconds(5));
    EXPECT_EQ(state_2.timeSinceLastKick(), Duration::fromMilliseconds(5));
    EXPECT_TRUE(state_1 == state_2);
    EXPECT_FALSE(state_1 != state_2);
}

TEST(RobotStateTest, compare_states_with_different_time_since)
{
    RobotState state_1(Point(1.1, -0.5), Vector(3, 0), Angle::quarter(),
                       AngularVelocity::zero());
    RobotState state_2(state_1);
    EXPECT_TRUE(state_1 == state_2);
    EXPECT_FALSE(state_1 != state_2);
    state_2.setTimeSinceLastChip(Duration::fromMilliseconds(5));
    EXPECT_TRUE(state_2.timeSinceLastChip());
    EXPECT_EQ(*state_2.timeSinceLastChip(), Duration::fromMilliseconds(5));
    EXPECT_FALSE(state_1 == state_2);
    EXPECT_TRUE(state_1 != state_2);
    state_2 = state_1;
    EXPECT_TRUE(state_1 == state_2);
    EXPECT_FALSE(state_1 != state_2);
    state_1.setTimeSinceLastKick(Duration::fromMilliseconds(5));
    EXPECT_TRUE(state_1.timeSinceLastKick());
    EXPECT_EQ(*state_1.timeSinceLastKick(), Duration::fromMilliseconds(5));
    EXPECT_FALSE(state_1 == state_2);
    EXPECT_TRUE(state_1 != state_2);
}

#include "software/world/ball_state.h"

#include <gtest/gtest.h>

TEST(BallStateTest, get_position)
{
    BallState ball_state(Point(1, -2.3), Vector(0, 0.4));
    EXPECT_EQ(ball_state.position(), Point(1, -2.3));
}

TEST(BallStateTest, get_velocity)
{
    BallState ball_state(Point(1, -2.3), Vector(0, 0.4));
    EXPECT_EQ(ball_state.velocity(), Vector(0, 0.4));
}

TEST(BallStateTest, compare_identical_states)
{
    BallState ball_state_1(Point(1, -2.3), Vector(0, 0.4));
    BallState ball_state_2(Point(1, -2.3), Vector(0, 0.4));
    EXPECT_TRUE(ball_state_1 == ball_state_2);
    EXPECT_FALSE(ball_state_1 != ball_state_2);
}

TEST(BallStateTest, compare_states_with_different_position)
{
    BallState ball_state_1(Point(1, -2.3), Vector(0, 0.4));
    BallState ball_state_2(Point(1, -2.4), Vector(0, 0.4));
    EXPECT_FALSE(ball_state_1 == ball_state_2);
    EXPECT_TRUE(ball_state_1 != ball_state_2);
}

TEST(BallStateTest, compare_states_with_different_velocities)
{
    BallState ball_state_1(Point(1, -2.3), Vector(0, 0.4));
    BallState ball_state_2(Point(1, -2.3), Vector(-1, 0.4));
    EXPECT_FALSE(ball_state_1 == ball_state_2);
    EXPECT_TRUE(ball_state_1 != ball_state_2);
}

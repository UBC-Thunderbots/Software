#include "software/world/timestamped_ball_state.h"

#include <gtest/gtest.h>

TEST(BallStateWithTimestampTest, get_timestamp)
{
    TimestampedBallState ball_state(Point(1, -2.3), Vector(0, 0.4),
                                    Timestamp::fromSeconds(0.65));
    EXPECT_EQ(ball_state.timestamp(), Timestamp::fromSeconds(0.65));
}

TEST(BallStateWithTimestampTest, get_ball_state)
{
    TimestampedBallState ball_state(Point(1, -2.3), Vector(0, 0.4),
                                    Timestamp::fromSeconds(0.65));
    EXPECT_EQ(ball_state.ballState(), BallState(Point(1, -2.3), Vector(0, 0.4)));
}

TEST(BallStateWithTimestampTest, compare_identical_states)
{
    TimestampedBallState ball_state_1(BallState(Point(1, -2.3), Vector(0, 0.4)),
                                      Timestamp::fromSeconds(0));
    TimestampedBallState ball_state_2(Point(1, -2.3), Vector(0, 0.4),
                                      Timestamp::fromSeconds(0));
    EXPECT_TRUE(ball_state_1 == ball_state_2);
    EXPECT_FALSE(ball_state_1 != ball_state_2);
}

TEST(BallStateWithTimestampTest, compare_states_with_different_position)
{
    TimestampedBallState ball_state_1(Point(1, -2.3), Vector(0, 0.4),
                                      Timestamp::fromSeconds(0));
    TimestampedBallState ball_state_2(Point(1, -2.4), Vector(0, 0.4),
                                      Timestamp::fromSeconds(0));
    EXPECT_FALSE(ball_state_1 == ball_state_2);
    EXPECT_TRUE(ball_state_1 != ball_state_2);
}

TEST(BallStateWithTimestampTest, compare_states_with_different_velocities)
{
    TimestampedBallState ball_state_1(Point(1, -2.3), Vector(0, 0.4),
                                      Timestamp::fromSeconds(0));
    TimestampedBallState ball_state_2(Point(1, -2.3), Vector(-1, 0.4),
                                      Timestamp::fromSeconds(0));
    EXPECT_FALSE(ball_state_1 == ball_state_2);
    EXPECT_TRUE(ball_state_1 != ball_state_2);
}

TEST(BallStateWithTimestampTest, compare_states_with_different_timestamps)
{
    TimestampedBallState ball_state_1(Point(1, -2.3), Vector(0, 0.4),
                                      Timestamp::fromSeconds(1));
    TimestampedBallState ball_state_2(Point(1, -2.3), Vector(0, 0.4),
                                      Timestamp::fromSeconds(0));
    EXPECT_TRUE(ball_state_1 == ball_state_2);
    EXPECT_FALSE(ball_state_1 != ball_state_2);
}

#include "software/world/ball_state_with_timestamp.h"
#include <gtest/gtest.h>

TEST(BallStateWithTimestampTest, get_position) {
    BallStateWithTimestamp ball_state(Point(1, -2.3), Vector(0, 0.4), Timestamp::fromSeconds(1.1));
    EXPECT_EQ(ball_state.position(), Point(1, -2.3));
}

TEST(BallStateWithTimestampTest, get_velocity) {
    BallStateWithTimestamp ball_state(BallState(Point(1, -2.3), Vector(0, 0.4)), Timestamp::fromSeconds(0.1));
    EXPECT_EQ(ball_state.velocity(), Vector(0, 0.4));
}

TEST(BallStateWithTimestampTest, get_timestamp) {
    BallStateWithTimestamp ball_state(Point(1, -2.3), Vector(0, 0.4), Timestamp::fromSeconds(0.65));
    EXPECT_EQ(ball_state.timestamp(), Timestamp::fromSeconds(0.65));
}

TEST(BallStateWithTimestampTest, get_ball_state) {
    BallStateWithTimestamp ball_state(Point(1, -2.3), Vector(0, 0.4), Timestamp::fromSeconds(0.65));
    EXPECT_EQ(ball_state.getBallState(), BallState(Point(1, -2.3), Vector(0, 0.4)));
}

TEST(BallStateWithTimestampTest, compare_identical_states) {
    BallStateWithTimestamp ball_state_1(BallState(Point(1, -2.3), Vector(0, 0.4)), Timestamp::fromSeconds(0));
    BallStateWithTimestamp ball_state_2(Point(1, -2.3), Vector(0, 0.4), Timestamp::fromSeconds(0));
    EXPECT_TRUE(ball_state_1 == ball_state_2);
    EXPECT_FALSE(ball_state_1 != ball_state_2);
}

TEST(BallStateWithTimestampTest, compare_states_with_different_position) {
    BallStateWithTimestamp ball_state_1(Point(1, -2.3), Vector(0, 0.4), Timestamp::fromSeconds(0));
    BallStateWithTimestamp ball_state_2(Point(1, -2.4), Vector(0, 0.4), Timestamp::fromSeconds(0));
    EXPECT_FALSE(ball_state_1 == ball_state_2);
    EXPECT_TRUE(ball_state_1 != ball_state_2);
}

TEST(BallStateWithTimestampTest, compare_states_with_different_velocities) {
    BallStateWithTimestamp ball_state_1(Point(1, -2.3), Vector(0, 0.4), Timestamp::fromSeconds(0));
    BallStateWithTimestamp ball_state_2(Point(1, -2.3), Vector(-1, 0.4), Timestamp::fromSeconds(0));
    EXPECT_FALSE(ball_state_1 == ball_state_2);
    EXPECT_TRUE(ball_state_1 != ball_state_2);
}

TEST(BallStateWithTimestampTest, compare_states_with_different_timestamps) {
    BallStateWithTimestamp ball_state_1(Point(1, -2.3), Vector(0, 0.4), Timestamp::fromSeconds(1));
    BallStateWithTimestamp ball_state_2(Point(1, -2.3), Vector(0, 0.4), Timestamp::fromSeconds(0));
    EXPECT_TRUE(ball_state_1 == ball_state_2);
    EXPECT_FALSE(ball_state_1 != ball_state_2);
}

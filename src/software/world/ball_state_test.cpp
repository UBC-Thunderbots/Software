#include "software/world/ball_state.h"

#include <gtest/gtest.h>

#include "proto/message_translation/tbots_protobuf.h"

TEST(BallStateTest, get_position)
{
    BallState ball_state(Point(1, -2.3), Vector(0, 0.4));
    EXPECT_EQ(Point(1, -2.3), ball_state.position());
}

TEST(BallStateTest, get_velocity)
{
    BallState ball_state(Point(1, -2.3), Vector(0, 0.4));
    EXPECT_EQ(Vector(0, 0.4), ball_state.velocity());
}

TEST(BallStateTest, get_default_constructed_distance_from_ground)
{
    BallState ball_state(Point(1, -2.3), Vector(0, 0.4));
    EXPECT_EQ(0, ball_state.distanceFromGround());
}

TEST(BallStateTest, get_distance_from_ground)
{
    BallState ball_state(Point(1, -2.3), Vector(0, 0.4), 0.54);
    EXPECT_EQ(0.54, ball_state.distanceFromGround());
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

TEST(BallStateTest, compare_states_with_different_distances_from_the_ground)
{
    BallState ball_state_1(Point(1, -2.3), Vector(0, 0.4), 0.1);
    BallState ball_state_2(Point(1, -2.3), Vector(0, 0.4), 0.2);
    EXPECT_FALSE(ball_state_1 == ball_state_2);
    EXPECT_TRUE(ball_state_1 != ball_state_2);
}

TEST(BallStateTest, construct_with_protobuf)
{
    BallState ball_state_1(Point(1, -2.3), Vector(0, 0.4), 0.1);
    Ball ball(ball_state_1, Timestamp());
    std::unique_ptr<TbotsProto::BallState> ball_state_proto = createBallStateProto(ball);
    BallState ball_state_2(*ball_state_proto);

    EXPECT_TRUE(ball_state_1 == ball_state_2);
}

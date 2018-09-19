#include "ai/world/ball.h"
#include <gtest/gtest.h>

using namespace std::chrono;

TEST(BallTest, construct_with_no_params)
{
    Ball ball = Ball();

    EXPECT_EQ(Point(), ball.position());
    EXPECT_EQ(Vector(), ball.velocity());
    // Can't compare timestamp values here because the ball and the expected timestamp
    // would not be createdx at the same time, and would not be equal. We could check
    // that the timestamps are within a certain threshold, but that is not robust and
    // makes the test dependant on the speed of the system executing it
}

TEST(BallTest, construct_with_params)
{
    auto current_time = steady_clock::now();

    Ball ball = Ball(Point(1, 2.3), Vector(-0.04, 0.0), current_time);

    EXPECT_EQ(Point(1, 2.3), ball.position());
    EXPECT_EQ(Vector(-0.04, 0.0), ball.velocity());
    EXPECT_EQ(current_time, ball.lastUpdateTimestamp());
}

TEST(BallTest, update_all_values)
{
    Ball ball = Ball();

    auto current_time = steady_clock::now();

    ball.update(Point(-4.23, 1.07), Vector(1, 2), current_time);

    EXPECT_EQ(Ball(Point(-4.23, 1.07), Vector(1, 2), current_time), ball);
}

TEST(BallTest, update_position_only)
{
    auto current_time = steady_clock::now();

    Ball ball = Ball(Point(-4.23, 1.07), Vector(1, 2), current_time);

    ball.update(Point(0.01, -99.8), ball.velocity(), current_time);

    EXPECT_EQ(Ball(Point(0.01, -99.8), Vector(1, 2), current_time), ball);
}

TEST(BallTest, update_velocity_only)
{
    auto current_time = steady_clock::now();

    Ball ball = Ball(Point(-4.23, 1.07), Vector(1, 2), current_time);

    ball.update(ball.position(), Vector(-0.0, -9.433), current_time);

    EXPECT_EQ(Ball(Point(-4.23, 1.07), Vector(-0.0, -9.433), current_time), ball);
}

TEST(BallTest, update_with_new_ball)
{
    auto current_time = steady_clock::now();

    Ball ball = Ball(Point(-4.23, 1.07), Vector(1, 2), current_time);

    Ball ball_update = Ball(Point(), Vector(-4.89, 3.1), current_time);

    ball.update(ball_update);

    EXPECT_EQ(ball_update, ball);
}

TEST(BallTest, update_values_with_past_timestamp)
{
    // TODO: Add unit tests to check for thrown exceptions when past timestamps are used
    // once https://github.com/UBC-Thunderbots/Software/issues/16 is done
}

TEST(BallTest, update_state_with_future_timestamp)
{
    auto current_time      = steady_clock::now();
    auto one_second_future = current_time + seconds(1);

    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12), current_time);

    ball.updateState(one_second_future);

    // A small distance to check that values are approximately equal
    double EPSILON = 1e-4;

    EXPECT_EQ(Point(-1.5, 6.88), ball.position());
    EXPECT_TRUE(Vector(-4.0717, -0.1086).isClose(ball.velocity(), EPSILON));
    EXPECT_EQ(one_second_future, ball.lastUpdateTimestamp());
}

TEST(BallTest, update_state_with_future_timestamp_2)
{
    auto current_time = steady_clock::now();
    auto future_time  = current_time + milliseconds(150);

    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12), current_time);

    ball.updateState(future_time);

    // A small distance to check that values are approximately equal
    double EPSILON = 1e-4;

    EXPECT_EQ(Point(2.325, 6.982), ball.position());
    EXPECT_TRUE(Vector(-4.4330, -0.1182).isClose(ball.velocity(), EPSILON));
    EXPECT_EQ(future_time, ball.lastUpdateTimestamp());
}

TEST(BallTest, update_state_with_past_timestamp)
{
    // TODO: Add unit tests to check for thrown exceptions when past timestamps are used
    // once https://github.com/UBC-Thunderbots/Software/issues/16 is done
}

TEST(BallTest, get_position_at_current_time)
{
    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12));

    EXPECT_EQ(Point(3, 7), ball.position());
}

TEST(BallTest, get_position_at_future_time)
{
    Ball ball = Ball(Point(), Vector(1, 2));

    EXPECT_EQ(Point(0.15, 0.3), ball.estimatePositionAtFutureTime(milliseconds(150)));
    EXPECT_EQ(Point(1, 2), ball.estimatePositionAtFutureTime(milliseconds(1000)));
    EXPECT_EQ(Point(2, 4), ball.estimatePositionAtFutureTime(milliseconds(2000)));
}

TEST(BallTest, get_position_at_future_time_2)
{
    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12));

    EXPECT_EQ(Point(2.325, 6.982), ball.estimatePositionAtFutureTime(milliseconds(150)));
    EXPECT_EQ(Point(-1.5, 6.88), ball.estimatePositionAtFutureTime(milliseconds(1000)));
    EXPECT_EQ(Point(-6, 6.76), ball.estimatePositionAtFutureTime(milliseconds(2000)));
}

TEST(BallTest, get_position_at_past_time)
{
    // TODO: Add unit tests to check for thrown exceptions when a negative value is passed
    // once https://github.com/UBC-Thunderbots/Software/issues/16 is done
}

TEST(BallTest, get_velocity_at_current_time)
{
    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12));

    EXPECT_EQ(Vector(-4.5, -0.12), ball.velocity());
}

TEST(BallTest, get_velocity_at_future_time)
{
    Ball ball = Ball(Point(), Vector(1, 2));

    // A small distance to check that values are approximately equal
    double EPSILON = 1e-4;

    EXPECT_TRUE(
        Vector(0.9851, 1.9702)
            .isClose(ball.estimateVelocityAtFutureTime(milliseconds(150)), EPSILON));
    EXPECT_TRUE(
        Vector(0.9048, 1.8097)
            .isClose(ball.estimateVelocityAtFutureTime(milliseconds(1000)), EPSILON));
    EXPECT_TRUE(
        Vector(0.8187, 1.6375)
            .isClose(ball.estimateVelocityAtFutureTime(milliseconds(2000)), EPSILON));
}

TEST(BallTest, get_velocity_at_future_time_2)
{
    // A small distance to check that values are approximately equal
    double EPSILON = 1e-4;

    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12));

    EXPECT_TRUE(
        Vector(-4.4330, -0.1182)
            .isClose(ball.estimateVelocityAtFutureTime(milliseconds(150)), EPSILON));
    EXPECT_TRUE(
        Vector(-4.0717, -0.1086)
            .isClose(ball.estimateVelocityAtFutureTime(milliseconds(1000)), EPSILON));
    EXPECT_TRUE(
        Vector(-3.6843, -0.0982)
            .isClose(ball.estimateVelocityAtFutureTime(milliseconds(2000)), EPSILON));
}

TEST(BallTest, get_velocity_at_past_time)
{
    // TODO: Add unit tests to check for thrown exceptions when a negative value is passed
    // once https://github.com/UBC-Thunderbots/Software/issues/16 is done
}

TEST(BallTest, get_last_update_timestamp)
{
    auto current_time       = steady_clock::now();
    auto half_second_future = current_time + milliseconds(500);

    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12), current_time);

    EXPECT_EQ(current_time, ball.lastUpdateTimestamp());

    ball.updateState(half_second_future);

    EXPECT_EQ(half_second_future, ball.lastUpdateTimestamp());
}

TEST(BallTest, get_last_update_timestamp_2)
{
    auto current_time        = steady_clock::now();
    auto nine_seconds_future = current_time + seconds(9);

    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12), current_time);

    EXPECT_EQ(current_time, ball.lastUpdateTimestamp());

    ball.updateState(nine_seconds_future);

    EXPECT_EQ(nine_seconds_future, ball.lastUpdateTimestamp());
}

TEST(BallTest, equality_operators)
{
    auto current_time = steady_clock::now();
    auto future_time  = current_time + milliseconds(123);

    Ball ball_0 = Ball();

    Ball ball_1 = Ball(Point(0.01, -0.0), Vector(), current_time);

    Ball ball_2 = Ball(Point(2, -3), Vector(0, 1), future_time);

    Ball ball_3 = Ball(Point(0.01, -0.0), Vector(), future_time);

    EXPECT_EQ(ball_0, ball_0);
    EXPECT_NE(ball_0, ball_1);
    EXPECT_NE(ball_0, ball_2);
    EXPECT_EQ(ball_1, ball_1);
    EXPECT_NE(ball_1, ball_2);
    EXPECT_EQ(ball_1, ball_3);
    EXPECT_NE(ball_2, ball_3);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

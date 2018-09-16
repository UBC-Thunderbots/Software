#include "ai/world/ball.h"
#include <gtest/gtest.h>

TEST(BallTest, construct_with_no_params)
{
    Ball ball = Ball();

    EXPECT_EQ(Point(), ball.position());
    EXPECT_EQ(Vector(), ball.velocity());
}

TEST(BallTest, construct_with_params)
{
    Ball ball = Ball(Point(1, 2.3), Vector(-0.04, 0.0));

    EXPECT_EQ(Point(1, 2.3), ball.position());
    EXPECT_EQ(Vector(-0.04, 0.0), ball.velocity());
}

TEST(BallTest, update_all_values)
{
    Ball ball = Ball();

    ball.update(Point(-4.23, 1.07), Vector(1, 2));

    EXPECT_EQ(Ball(Point(-4.23, 1.07), Vector(1, 2)), ball);
}

TEST(BallTest, update_position_only)
{
    Ball ball = Ball(Point(-4.23, 1.07), Vector(1, 2));

    ball.update(Point(0.01, -99.8), ball.velocity());

    EXPECT_EQ(Ball(Point(0.01, -99.8), Vector(1, 2)), ball);
}

TEST(BallTest, update_velocity_only)
{
    Ball ball = Ball(Point(-4.23, 1.07), Vector(1, 2));

    ball.update(ball.position(), Vector(-0.0, -9.433));

    EXPECT_EQ(Ball(Point(-4.23, 1.07), Vector(-0.0, -9.433)), ball);
}

TEST(BallTest, update_with_new_ball)
{
    Ball ball = Ball(Point(-4.23, 1.07), Vector(1, 2));

    ball.update(Ball(Point(), Vector(-4.89, 3.1)));

    EXPECT_EQ(Ball(Point(), Vector(-4.89, 3.1)), ball);
}

TEST(BallTest, get_position_at_current_time)
{
    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12));

    EXPECT_EQ(Point(3, 7), ball.position());
}

TEST(BallTest, predict_future_position)
{
    Ball ball = Ball(Point(), Vector(1, 2));

    EXPECT_EQ(Point(1, 2), ball.estimatePositionAtFutureTime(1));
    EXPECT_EQ(Point(2, 4), ball.estimatePositionAtFutureTime(2));
    EXPECT_EQ(Point(0.15, 0.3), ball.estimatePositionAtFutureTime(0.15));

    ball = Ball(Point(3, 7), Vector(-4.5, -0.12));

    EXPECT_EQ(Point(-1.5, 6.88), ball.estimatePositionAtFutureTime(1));
    EXPECT_EQ(Point(-6, 6.76), ball.estimatePositionAtFutureTime(2));
    EXPECT_EQ(Point(2.325, 6.982), ball.estimatePositionAtFutureTime(0.15));
}

TEST(BallTest, get_velocity_at_current_time)
{
    Ball ball = Ball(Point(3, 7), Vector(-4.5, -0.12));

    EXPECT_EQ(Vector(-4.5, -0.12), ball.velocity());
}

TEST(BallTest, predict_future_velocity)
{
    Ball ball = Ball(Point(), Vector(1, 2));

    // A small distance to check that values are approximately equal
    double EPSILON = 1e-4;

    EXPECT_TRUE(
        Vector(0.9851, 1.9702).isClose(ball.estimateVelocityAtFutureTime(0.15), EPSILON));
    EXPECT_TRUE(
        Vector(0.9048, 1.8097).isClose(ball.estimateVelocityAtFutureTime(1), EPSILON));
    EXPECT_TRUE(
        Vector(0.8187, 1.6375).isClose(ball.estimateVelocityAtFutureTime(2), EPSILON));

    ball = Ball(Point(3, 7), Vector(-4.5, -0.12));

    EXPECT_TRUE(Vector(-4.4330, -0.1182)
                    .isClose(ball.estimateVelocityAtFutureTime(0.15), EPSILON));
    EXPECT_TRUE(
        Vector(-4.0717, -0.1086).isClose(ball.estimateVelocityAtFutureTime(1), EPSILON));
    EXPECT_TRUE(
        Vector(-3.6843, -0.0982).isClose(ball.estimateVelocityAtFutureTime(2), EPSILON));
}

TEST(BallTest, equality_operators)
{
    Ball ball_0 = Ball();

    Ball ball_1 = Ball(Point(0.01, -0.0), Vector());

    Ball ball_2 = Ball(Point(2, -3), Vector(0, 1));

    Ball ball_3 = Ball(Point(0.01, -0.0), Vector());

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

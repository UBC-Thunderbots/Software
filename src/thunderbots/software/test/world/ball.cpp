#include "ai/world/ball.h"
#include <gtest/gtest.h>

TEST(BallTest, construction)
{
    Ball ball = Ball();

    EXPECT_EQ(Point(), ball.position());
    EXPECT_EQ(Vector(), ball.velocity());
}

TEST(BallTest, update_and_accessors)
{
    Ball ball = Ball();

    ball.update(Point(-4.23, 1.07), Vector(1, 2));

    EXPECT_EQ(Point(-4.23, 1.07), ball.position());
    EXPECT_EQ(Vector(1, 2), ball.velocity());

    ball.update(Point(), Vector(0.01, 3.4));

    EXPECT_EQ(Point(), ball.position());
    EXPECT_EQ(Vector(0.01, 3.4), ball.velocity());
}

TEST(BallTest, equality_operators)
{
    Ball ball_0 = Ball();

    Ball ball_1 = Ball();
    ball_1.update(Point(0.01, -0.0), Vector());

    Ball ball_2 = Ball();
    ball_2.update(Point(2, -3), Vector(0, 1));

    Ball ball_3 = Ball();
    ball_3.update(Point(0.01, -0.0), Vector());

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

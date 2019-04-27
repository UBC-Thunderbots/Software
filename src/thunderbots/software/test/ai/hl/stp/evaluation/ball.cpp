#include "ai/hl/stp/evaluation/ball.h"

#include <gtest/gtest.h>

#include "test/test_util/test_util.h"

TEST(BallInFriendlyHalfTest, ball_barely_in_friendly_half)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball   = Ball(Point(-0.01, 1), Point(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_TRUE(Evaluation::ballInFriendlyHalf(field, ball));
}

TEST(BallInFriendlyHalfTest, ball_at_friendly_corner)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball =
        Ball(field.friendlyCornerNeg(), Point(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_TRUE(Evaluation::ballInFriendlyHalf(field, ball));
}

TEST(BallInFriendlyHalfTest, ball_barely_in_enemy_half)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball   = Ball(Point(0.03, -3), Point(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_FALSE(Evaluation::ballInFriendlyHalf(field, ball));
}

TEST(BallInFriendlyHalfTest, ball_at_enemy_goal)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball   = Ball(field.enemyGoal(), Point(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_FALSE(Evaluation::ballInFriendlyHalf(field, ball));
}

TEST(BallInEnemyHalfTest, ball_barely_in_enemy_half)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball   = Ball(Point(0.01, 1), Point(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_TRUE(Evaluation::ballInEnemyHalf(field, ball));
}

TEST(BallInEnemyHalfTest, ball_at_enemy_corner)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball = Ball(field.enemyCornerNeg(), Point(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_TRUE(Evaluation::ballInEnemyHalf(field, ball));
}

TEST(BallInEnemyHalfTest, ball_barely_in_friendly_half)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball   = Ball(Point(-0.03, -3), Point(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_FALSE(Evaluation::ballInEnemyHalf(field, ball));
}

TEST(BallInEnemyHalfTest, ball_at_friendly_goal)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball   = Ball(field.friendlyGoal(), Point(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_FALSE(Evaluation::ballInEnemyHalf(field, ball));
}

class BallPositionsInFriendlyCorner : public ::testing::TestWithParam<Point>
{
};

class BallPositionsNotInFriendlyCorner : public ::testing::TestWithParam<Point>
{
};

TEST_P(BallPositionsInFriendlyCorner, in_corner_test)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball   = Ball(GetParam(), Point(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_FALSE(Evaluation::ballInEnemyCorner(field, ball, 2.0));
    EXPECT_TRUE(Evaluation::ballInFriendlyCorner(field, ball, 2.0));

    EXPECT_FALSE(Evaluation::ballInEnemyCorner(field, ball, 1.0));
    EXPECT_TRUE(Evaluation::ballInFriendlyCorner(field, ball, 1.0));
}

INSTANTIATE_TEST_CASE_P(Positions, BallPositionsInFriendlyCorner,
                        ::testing::Values(Point(-4.5, 3), Point(-4.5, -3), Point(-4.0, 3),
                                          Point(-4.0, -3)));

// Field field = Field(9.0, 6.0, 1.0, 2.0, 1.0, 0.3, 0.5);
TEST_P(BallPositionsNotInFriendlyCorner, out_of_corner_test)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball   = Ball(GetParam(), Point(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_TRUE(Evaluation::ballInEnemyCorner(field, ball, 2.0));
    EXPECT_FALSE(Evaluation::ballInFriendlyCorner(field, ball, 2.0));

    EXPECT_TRUE(Evaluation::ballInEnemyCorner(field, ball, 1.0));
    EXPECT_FALSE(Evaluation::ballInFriendlyCorner(field, ball, 1.0));
}

INSTANTIATE_TEST_CASE_P(Positions, BallPositionsNotInFriendlyCorner,
                        ::testing::Values(Point(4.5, 3), Point(4.5, -3), Point(4.0, 3),
                                          Point(4.0, -3)));

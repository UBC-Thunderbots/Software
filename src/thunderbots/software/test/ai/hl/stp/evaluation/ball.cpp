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

class BallPositionsInFriendlyCornerOffField : public ::testing::TestWithParam<Point>
{
};

class BallPositionsInFriendlyCornerOnField : public ::testing::TestWithParam<Point>
{
};

class BallPositionsInEnemyCornerOnField : public ::testing::TestWithParam<Point>
{
};

class BallPositionsInEnemyCornerOffField : public ::testing::TestWithParam<Point>
{
};

TEST_P(BallPositionsInFriendlyCornerOnField, in_corner_inside_field)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball   = Ball(GetParam(), Point(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_FALSE(Evaluation::ballInEnemyCorner(field, ball, 2.0));
    EXPECT_TRUE(Evaluation::ballInFriendlyCorner(field, ball, 2.0));

    EXPECT_FALSE(Evaluation::ballInEnemyCorner(field, ball, 1.0));
    EXPECT_TRUE(Evaluation::ballInFriendlyCorner(field, ball, 1.0));
}

INSTANTIATE_TEST_CASE_P(Positions, BallPositionsInFriendlyCornerOnField,
                        ::testing::Values(Point(-4.5, 3), Point(-4.5, -3), Point(-4.0, 3),
                                          Point(-4.0, -3)));

TEST_P(BallPositionsInFriendlyCornerOffField, in_corner_outside_field)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball   = Ball(GetParam(), Point(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_FALSE(Evaluation::ballInFriendlyCorner(field, ball, 1.0));
    EXPECT_FALSE(Evaluation::ballInEnemyCorner(field, ball, 1.0));
}

INSTANTIATE_TEST_CASE_P(Positions, BallPositionsInFriendlyCornerOffField,
                        ::testing::Values(Point(-4.6, 3), Point(-4.6, -3), Point(-5.0, 3),
                                          Point(-5.0, -3), Point(-4.5, -3.1),
                                          Point(-4.0, -3.1)));

TEST_P(BallPositionsInEnemyCornerOnField, in_corner_inside_field)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball   = Ball(GetParam(), Point(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_TRUE(Evaluation::ballInEnemyCorner(field, ball, 2.0));
    EXPECT_FALSE(Evaluation::ballInFriendlyCorner(field, ball, 2.0));

    EXPECT_TRUE(Evaluation::ballInEnemyCorner(field, ball, 1.0));
    EXPECT_FALSE(Evaluation::ballInFriendlyCorner(field, ball, 1.0));
}

INSTANTIATE_TEST_CASE_P(Positions, BallPositionsInEnemyCornerOnField,
                        ::testing::Values(Point(4.5, 3), Point(4.5, -3), Point(4.0, 3),
                                          Point(4.0, -3)));

TEST_P(BallPositionsInEnemyCornerOffField, in_corner_outside_field)
{
    Field field = ::Test::TestUtil::createSSLDivBField();
    Ball ball   = Ball(GetParam(), Point(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_FALSE(Evaluation::ballInEnemyCorner(field, ball, 1.0));
    EXPECT_FALSE(Evaluation::ballInFriendlyCorner(field, ball, 1.0));
}

INSTANTIATE_TEST_CASE_P(Positions, BallPositionsInEnemyCornerOffField,
                        ::testing::Values(Point(4.6, 3), Point(4.6, -3), Point(5.0, 3),
                                          Point(5.0, -3), Point(4.5, -3.1),
                                          Point(4.0, -3.1)));

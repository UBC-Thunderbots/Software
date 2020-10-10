#include "software/ai/evaluation/ball.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

TEST(BallInFriendlyHalfTest, ball_barely_in_friendly_half)
{
    Field field = Field::createSSLDivisionBField();
    Ball ball   = Ball(Point(-0.01, 1), Vector(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_TRUE(ballInFriendlyHalf(field, ball));
}

TEST(BallInFriendlyHalfTest, ball_at_friendly_corner)
{
    Field field = Field::createSSLDivisionBField();
    Ball ball =
        Ball(field.friendlyCornerNeg(), Vector(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_TRUE(ballInFriendlyHalf(field, ball));
}

TEST(BallInFriendlyHalfTest, ball_barely_in_enemy_half)
{
    Field field = Field::createSSLDivisionBField();
    Ball ball   = Ball(Point(0.03, -3), Vector(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_FALSE(ballInFriendlyHalf(field, ball));
}

TEST(BallInFriendlyHalfTest, ball_at_enemy_goal)
{
    Field field = Field::createSSLDivisionBField();
    Ball ball =
        Ball(field.enemyGoalCenter(), Vector(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_FALSE(ballInFriendlyHalf(field, ball));
}

TEST(BallInEnemyHalfTest, ball_barely_in_enemy_half)
{
    Field field = Field::createSSLDivisionBField();
    Ball ball   = Ball(Point(0.01, 1), Vector(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_TRUE(ballInEnemyHalf(field, ball));
}

TEST(BallInEnemyHalfTest, ball_at_enemy_corner)
{
    Field field = Field::createSSLDivisionBField();
    Ball ball =
        Ball(field.enemyCornerNeg(), Vector(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_TRUE(ballInEnemyHalf(field, ball));
}

TEST(BallInEnemyHalfTest, ball_barely_in_friendly_half)
{
    Field field = Field::createSSLDivisionBField();
    Ball ball   = Ball(Point(-0.03, -3), Vector(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_FALSE(ballInEnemyHalf(field, ball));
}

TEST(BallInEnemyHalfTest, ball_at_friendly_goal)
{
    Field field = Field::createSSLDivisionBField();
    Ball ball =
        Ball(field.friendlyGoalCenter(), Vector(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_FALSE(ballInEnemyHalf(field, ball));
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
    Field field = Field::createSSLDivisionBField();
    Ball ball   = Ball(GetParam(), Vector(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_FALSE(ballInEnemyCorner(field, ball, 2.0));
    EXPECT_TRUE(ballInFriendlyCorner(field, ball, 2.0));

    EXPECT_FALSE(ballInEnemyCorner(field, ball, 1.0));
    EXPECT_TRUE(ballInFriendlyCorner(field, ball, 1.0));
}

INSTANTIATE_TEST_CASE_P(Positions, BallPositionsInFriendlyCornerOnField,
                        ::testing::Values(Point(-4.5, 3), Point(-4.5, -3), Point(-4.0, 3),
                                          Point(-4.0, -3)));

TEST_P(BallPositionsInFriendlyCornerOffField, in_corner_outside_field)
{
    Field field = Field::createSSLDivisionBField();
    Ball ball   = Ball(GetParam(), Vector(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_FALSE(ballInFriendlyCorner(field, ball, 1.0));
    EXPECT_FALSE(ballInEnemyCorner(field, ball, 1.0));
}

INSTANTIATE_TEST_CASE_P(Positions, BallPositionsInFriendlyCornerOffField,
                        ::testing::Values(Point(-4.6, 3), Point(-4.6, -3), Point(-5.0, 3),
                                          Point(-5.0, -3), Point(-4.5, -3.1),
                                          Point(-4.0, -3.1)));

TEST_P(BallPositionsInEnemyCornerOnField, in_corner_inside_field)
{
    Field field = Field::createSSLDivisionBField();
    Ball ball   = Ball(GetParam(), Vector(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_TRUE(ballInEnemyCorner(field, ball, 2.0));
    EXPECT_FALSE(ballInFriendlyCorner(field, ball, 2.0));

    EXPECT_TRUE(ballInEnemyCorner(field, ball, 1.0));
    EXPECT_FALSE(ballInFriendlyCorner(field, ball, 1.0));
}

INSTANTIATE_TEST_CASE_P(Positions, BallPositionsInEnemyCornerOnField,
                        ::testing::Values(Point(4.5, 3), Point(4.5, -3), Point(4.0, 3),
                                          Point(4.0, -3)));

TEST_P(BallPositionsInEnemyCornerOffField, in_corner_outside_field)
{
    Field field = Field::createSSLDivisionBField();
    Ball ball   = Ball(GetParam(), Vector(0, 0), Timestamp::fromMilliseconds(0));

    EXPECT_FALSE(ballInEnemyCorner(field, ball, 1.0));
    EXPECT_FALSE(ballInFriendlyCorner(field, ball, 1.0));
}

INSTANTIATE_TEST_CASE_P(Positions, BallPositionsInEnemyCornerOffField,
                        ::testing::Values(Point(4.6, 3), Point(4.6, -3), Point(5.0, 3),
                                          Point(5.0, -3), Point(4.5, -3.1),
                                          Point(4.0, -3.1)));
TEST(HasBallBeenKickedTest, ball_over_speed_threshold_and_no_direction_difference)
{
    Ball ball({0, 0}, {5, 5}, Timestamp::fromSeconds(0));

    Vector expected_direction(4, 4);

    EXPECT_TRUE(hasBallBeenKicked(ball, expected_direction.orientation()));
}

TEST(HasBallBeenKickedTest, ball_under_speed_threshold_and_no_direction_difference)
{
    Ball ball({5, 2}, {0, 0.2}, Timestamp::fromSeconds(0));

    Angle expected_direction = Angle::fromDegrees(90);

    EXPECT_FALSE(hasBallBeenKicked(ball, expected_direction));
}

TEST(HasBallBeenKickedTest,
     ball_under_optional_speed_threshold_and_small_direction_difference)
{
    Ball ball({0, 0}, {3, 0}, Timestamp::fromSeconds(0));

    Angle expected_direction = Angle::fromRadians(0);

    EXPECT_FALSE(hasBallBeenKicked(ball, expected_direction, 5));
}

TEST(HasBallBeenKickedTest,
     ball_over_speed_threshold_and_small_negative_direction_difference)
{
    Ball ball({0, 1}, {3, 0}, Timestamp::fromSeconds(0));

    Angle expected_direction = Angle::fromDegrees(-15);

    EXPECT_TRUE(hasBallBeenKicked(ball, expected_direction));
}

TEST(HasBallBeenKickedTest,
     ball_over_speed_threshold_and_small_positive_direction_difference)
{
    Ball ball({0, 1}, {3, 0}, Timestamp::fromSeconds(0));

    Angle expected_direction = Angle::fromDegrees(15);

    EXPECT_TRUE(hasBallBeenKicked(ball, expected_direction));
}

TEST(HasBallBeenKickedTest, ball_over_speed_threshold_and_large_direction_difference)
{
    Ball ball({-5, 2}, {-3, 0}, Timestamp::fromSeconds(0));

    Angle expected_direction = Angle::fromRadians(0);

    EXPECT_FALSE(hasBallBeenKicked(ball, expected_direction));
}

TEST(HasBallBeenKickedTest, ball_under_speed_threshold_and_large_direction_difference)
{
    Ball ball({2, 2}, {0.2, 0.2}, Timestamp::fromSeconds(0));

    Angle expected_direction = Angle::fromDegrees(90);

    EXPECT_FALSE(hasBallBeenKicked(ball, expected_direction));
}

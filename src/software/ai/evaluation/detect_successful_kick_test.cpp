#include "software/ai/evaluation/detect_successful_kick.h"

#include <gtest/gtest.h>


TEST(DetectSuccessfulKickTest, ball_over_speed_threshold_and_no_direction_difference)
{
    Ball ball({0, 0}, {5, 5}, Timestamp::fromSeconds(0));

    Vector expected_direction(4, 4);

    EXPECT_TRUE(hasBallBeenKicked(ball, expected_direction.orientation()));
}

TEST(DetectSuccessfulKickTest, ball_under_speed_threshold_and_no_direction_difference)
{
    Ball ball({5, 2}, {0, 0.2}, Timestamp::fromSeconds(0));

    Angle expected_direction = Angle::fromDegrees(90);

    EXPECT_FALSE(hasBallBeenKicked(ball, expected_direction));
}

TEST(DetectSuccessfulKickTest,
     ball_over_speed_threshold_and_small_negative_direction_difference)
{
    Ball ball({0, 1}, {3, 0}, Timestamp::fromSeconds(0));

    Angle expected_direction = Angle::fromDegrees(-15);

    EXPECT_TRUE(hasBallBeenKicked(ball, expected_direction));
}

TEST(DetectSuccessfulKickTest,
     ball_over_speed_threshold_and_small_positive_direction_difference)
{
    Ball ball({0, 1}, {3, 0}, Timestamp::fromSeconds(0));

    Angle expected_direction = Angle::fromDegrees(15);

    EXPECT_TRUE(hasBallBeenKicked(ball, expected_direction));
}

TEST(DetectSuccessfulKickTest, ball_over_speed_threshold_and_exceeds_direction_threshold)
{
    Ball ball({-5, 2}, {-3, 0}, Timestamp::fromSeconds(0));

    Angle expected_direction = Angle::fromRadians(0);

    EXPECT_FALSE(hasBallBeenKicked(ball, expected_direction));
}

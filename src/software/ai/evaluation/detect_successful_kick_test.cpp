#include "software/ai/evaluation/detect_successful_kick.h"

#include <gtest/gtest.h>


TEST(DetectSuccessfulKickTest, ball_over_speed_threshold_and_no_direction_difference)
{
    Ball ball({0, 0}, {5, 5}, Timestamp::fromSeconds(0));

    Vector expected_direction(4, 4);

    EXPECT_TRUE(successfulKickDetected(ball, expected_direction));
}

TEST(DetectSuccessfulKickTest, ball_under_speed_threshold_and_no_direction_difference)
{
    Ball ball({5, 2}, {0, 0.2}, Timestamp::fromSeconds(0));

    Angle angle               = angle.fromDegrees(90);
    Vector expected_direction = Vector::createFromAngle(angle);

    EXPECT_FALSE(successfulKickDetected(ball, expected_direction));
}

TEST(DetectSuccessfulKickTest,
     ball_over_speed_threshold_and_small_negative_direction_difference)
{
    Ball ball({0, 1}, {3, 0}, Timestamp::fromSeconds(0));

    Angle angle               = angle.fromDegrees(-15);
    Vector expected_direction = Vector::createFromAngle(angle);

    EXPECT_TRUE(successfulKickDetected(ball, expected_direction));
}

TEST(DetectSuccessfulKickTest, ball_over_speed_threshold_and_exceeds_direction_threshold)
{
    Ball ball({-5, 2}, {-3, 0}, Timestamp::fromSeconds(0));

    Angle angle               = angle.fromRadians(0);
    Vector expected_direction = Vector::createFromAngle(angle);

    EXPECT_FALSE(successfulKickDetected(ball, expected_direction));
}

#include "software/ai/evaluation/detect_successful_kick.h"

#include <gtest/gtest.h>


TEST(DetectSuccessfulKickTest, ball_over_speed_threshold_and_no_angle_difference_expect_success) {
    Ball ball({0, 0}, {5, 5}, Timestamp::fromSeconds(0));

    Angle angle = angle.fromDegrees(45);
    Vector expected_direction = Vector::createFromAngle(angle);

    EXPECT_TRUE(successfulKickDetected(ball, expected_direction));
}

TEST(DetectSuccessfulKickTest, ball_over_speed_threshold_and_no_angle_difference_expect_success) {
    Ball ball({0, 0}, {5, 5}, Timestamp::fromSeconds(0));

    Angle angle = angle.fromDegrees(45);
    Vector expected_direction = Vector::createFromAngle(angle);

    EXPECT_TRUE(successfulKickDetected(ball, expected_direction));
}

TEST(DetectSuccessfulKickTest, ball_over_speed_threshold_and_no_angle_difference_expect_success) {
    Ball ball({0, 0}, {5, 5}, Timestamp::fromSeconds(0));

    Angle angle = angle.fromDegrees(45);
    Vector expected_direction = Vector::createFromAngle(angle);

    EXPECT_TRUE(successfulKickDetected(ball, expected_direction));
}

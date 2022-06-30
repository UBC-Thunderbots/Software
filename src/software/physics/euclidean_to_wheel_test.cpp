#include "software/physics/euclidean_to_wheel.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

class EuclideanToWheelTest : public ::testing::Test
{
   protected:
    EuclideanToWheelTest() = default;
    EuclideanSpace_t target_euclidean_velocity{};
    WheelSpace_t expected_wheel_speeds{};

    EuclideanToWheel euclidean_to_four_wheel =
        EuclideanToWheel(create2021RobotConstants());
};

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_zero)
{
    // test +/right
    target_euclidean_velocity = {0, 0, 0};
    expected_wheel_speeds     = {0, 0, 0, 0};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getTargetWheelSpeeds(target_euclidean_velocity),
        0.001));
}

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_x)
{
    // test +/right
    target_euclidean_velocity = {1, 0, 0};
    expected_wheel_speeds     = {-0.5308, -0.5308, 0.7198, 0.7198};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getTargetWheelSpeeds(target_euclidean_velocity),
        0.001));

    // test -/left
    target_euclidean_velocity = {-1, 0, 0};
    expected_wheel_speeds     = {0.5308, 0.5308, -0.7198, -0.7198};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getTargetWheelSpeeds(target_euclidean_velocity),
        0.001));
}

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_y)
{
    // test +/forwards
    target_euclidean_velocity = {0, 1, 0};
    expected_wheel_speeds     = {0.8475, -0.8475, -0.6942, 0.6942};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getTargetWheelSpeeds(target_euclidean_velocity),
        0.001));

    // test -/backwards
    target_euclidean_velocity = {0, -1, 0};
    expected_wheel_speeds     = {-0.8475, 0.8475, 0.6942, -0.6942};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getTargetWheelSpeeds(target_euclidean_velocity),
        0.001));
}

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_w)
{
    // test +/forwards
    target_euclidean_velocity = {0, 0, 1};
    expected_wheel_speeds     = {1, 1, 1, 1};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getTargetWheelSpeeds(target_euclidean_velocity),
        0.001));

    // test -/backwards
    target_euclidean_velocity = {0, 0, -1};
    expected_wheel_speeds     = {-1, -1, -1, -1};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getTargetWheelSpeeds(target_euclidean_velocity),
        0.001));
}

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_all)
{
    // test +/forwards
    target_euclidean_velocity = {1, 1, 1};
    expected_wheel_speeds     = {1.3167, -0.3783, 1.0257, 2.4140};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getTargetWheelSpeeds(target_euclidean_velocity),
        0.001));
}


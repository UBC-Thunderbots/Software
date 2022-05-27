#include "software/physics/euclidean_to_wheel.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

class EuclideanToWheelTest : public ::testing::Test
{
   protected:
    EuclideanToWheelTest() = default;
    WheelSpace_t current_wheel_speeds{};
    EuclideanSpace_t target_euclidean_velocity{};
    WheelSpace_t expected_wheel_speeds{};

    EuclideanToWheel euclidean_to_four_wheel =
        EuclideanToWheel(200, create2022RobotConstants());
};

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_zero)
{
    // test +/right
    current_wheel_speeds      = {0, 0, 0, 0};
    target_euclidean_velocity = {0, 0, 0};
    expected_wheel_speeds     = {0, 0, 0, 0};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getTargetWheelSpeeds(target_euclidean_velocity,
                                                     current_wheel_speeds),
        0.001));
}

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_x)
{
    // test +/right
    current_wheel_speeds      = {0, 0, 0, 0};
    target_euclidean_velocity = {1, 0, 0};
    expected_wheel_speeds     = {-2.2584, -2.2584, 4.0112, 4.0112};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getTargetWheelSpeeds(target_euclidean_velocity,
                                                     current_wheel_speeds),
        0.001));

    // test -/left
    current_wheel_speeds      = {0, 0, 0, 0};
    target_euclidean_velocity = {-1, 0, 0};
    expected_wheel_speeds     = {2.2584, 2.2584, -4.0112, -4.0112};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getTargetWheelSpeeds(target_euclidean_velocity,
                                                     current_wheel_speeds),
        0.001));
}

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_y)
{
    // test +/forwards
    current_wheel_speeds      = {0, 0, 0, 0};
    target_euclidean_velocity = {0, 1, 0};
    expected_wheel_speeds     = {2.1226, -2.1226, -2.7766, 2.7766};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getTargetWheelSpeeds(target_euclidean_velocity,
                                                     current_wheel_speeds),
        0.001));

    // test -/backwards
    current_wheel_speeds      = {0, 0, 0, 0};
    target_euclidean_velocity = {0, -1, 0};
    expected_wheel_speeds     = {-2.1226, 2.1226, 2.7766, -2.7766};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getTargetWheelSpeeds(target_euclidean_velocity,
                                                     current_wheel_speeds),
        0.001));
}

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_w)
{
    // test +/forwards
    current_wheel_speeds      = {0, 0, 0, 0};
    target_euclidean_velocity = {0, 0, 1};
    expected_wheel_speeds     = {0.0918, 0.0918, 0.0885, 0.0885};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getTargetWheelSpeeds(target_euclidean_velocity,
                                                     current_wheel_speeds),
        0.001));

    // test -/backwards
    current_wheel_speeds      = {0, 0, 0, 0};
    target_euclidean_velocity = {0, 0, -1};
    expected_wheel_speeds     = {-0.0918, -0.0918, -0.0885, -0.0885};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getTargetWheelSpeeds(target_euclidean_velocity,
                                                     current_wheel_speeds),
        0.001));
}

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_all)
{
    // test +/forwards
    current_wheel_speeds      = {0, 0, 0, 0};
    target_euclidean_velocity = {1, 1, 1};
    expected_wheel_speeds     = {-0.0440, -4.2893, 1.3231, 6.8763};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getTargetWheelSpeeds(target_euclidean_velocity,
                                                     current_wheel_speeds),
        0.001));
}

TEST_F(EuclideanToWheelTest, test_sanity_check_conversion_is_linear)
{
    current_wheel_speeds      = {1, 2, 3, 5};
    target_euclidean_velocity = {3, 1, 5};

    auto result = euclidean_to_four_wheel.getTargetWheelSpeeds(target_euclidean_velocity,
                                                               current_wheel_speeds);

    current_wheel_speeds      = {100, 200, 300, 500};
    target_euclidean_velocity = {300, 100, 500};

    auto scaled_result = euclidean_to_four_wheel.getTargetWheelSpeeds(
        target_euclidean_velocity, current_wheel_speeds);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(result * 100, scaled_result, 0.001));
}

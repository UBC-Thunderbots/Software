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
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity), 0.001));
}

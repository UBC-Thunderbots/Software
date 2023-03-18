#include "software/physics/euclidean_to_three_wheel.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

class EuclideanToThreeWheelTest : public ::testing::Test
{
   protected:
    EuclideanToThreeWheelTest() = default;
    EuclideanSpace_t target_euclidean_velocity{};
    ThreeWheelSpace_t expected_wheel_speeds{};
    ThreeWheelSpace_t calculated_wheel_speeds{};
    double robot_radius = create2021RobotConstants().robot_radius_m;

    EuclideanToThreeWheel euclidean_to_three_wheel =
            EuclideanToThreeWheel(create2021RobotConstants());
};

TEST_F(EuclideanToThreeWheelTest, test_target_wheel_speeds_zero)
{
    target_euclidean_velocity = {0, 0, 0};
    expected_wheel_speeds     = {0, 0, 0};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_three_wheel.getWheelVelocity(target_euclidean_velocity), 0.001));
}

// Note: The tests below assume that counter-clockwise motor rotation is positive
// velocity, and vise-versa.
TEST_F(EuclideanToThreeWheelTest, test_target_wheel_speeds_positive_x)
{
    // Test +x/right
    target_euclidean_velocity = {1, 0, 0};
    calculated_wheel_speeds =
        euclidean_to_three_wheel.getWheelVelocity(target_euclidean_velocity);

    // Front wheels must be - velocity, back wheels must be + velocity.
    EXPECT_LT(calculated_wheel_speeds[FRONT_WHEEL_SPACE_INDEX], 0);
    EXPECT_GT(calculated_wheel_speeds[LEFT_WHEEL_SPACE_INDEX], 0);
    EXPECT_GT(calculated_wheel_speeds[RIGHT_WHEEL_SPACE_INDEX], 0);
}

TEST_F(EuclideanToThreeWheelTest, test_target_wheel_speeds_negative_x)
{
    // Test -x/left
    target_euclidean_velocity = {-1, 0, 0};
    calculated_wheel_speeds =
        euclidean_to_three_wheel.getWheelVelocity(target_euclidean_velocity);

    // Front wheels must be + velocity, back wheels must be - velocity.
    EXPECT_GT(calculated_wheel_speeds[FRONT_WHEEL_SPACE_INDEX], 0);
    EXPECT_LT(calculated_wheel_speeds[LEFT_WHEEL_SPACE_INDEX], 0);
    EXPECT_LT(calculated_wheel_speeds[RIGHT_WHEEL_SPACE_INDEX], 0);
}

TEST_F(EuclideanToThreeWheelTest, test_target_wheel_speeds_positive_y)
{
    // Test +y/forwards
    target_euclidean_velocity = {0, 1, 0};
    calculated_wheel_speeds =
        euclidean_to_three_wheel.getWheelVelocity(target_euclidean_velocity);

    // Right wheels must be + velocity, Left wheels must be - velocity.
    EXPECT_GT(calculated_wheel_speeds[RIGHT_WHEEL_SPACE_INDEX], 0);
    EXPECT_LT(calculated_wheel_speeds[LEFT_WHEEL_SPACE_INDEX], 0);
    EXPECT_EQ(calculated_wheel_speeds[FRONT_WHEEL_SPACE_INDEX], 0);

    // Right wheel must have same velocity magnitude as left wheel, but opposite sign.
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[RIGHT_WHEEL_SPACE_INDEX],
                     -calculated_wheel_speeds[LEFT_WHEEL_SPACE_INDEX]);
}

TEST_F(EuclideanToThreeWheelTest, test_target_wheel_speeds_negative_y)
{
    // Test -y/backwards
    target_euclidean_velocity = {0, -1, 0};
    calculated_wheel_speeds =
        euclidean_to_three_wheel.getWheelVelocity(target_euclidean_velocity);

    // Right wheels must be + velocity, Left wheels must be - velocity.
    EXPECT_LT(calculated_wheel_speeds[RIGHT_WHEEL_SPACE_INDEX], 0);
    EXPECT_GT(calculated_wheel_speeds[LEFT_WHEEL_SPACE_INDEX], 0);
    EXPECT_EQ(calculated_wheel_speeds[FRONT_WHEEL_SPACE_INDEX], 0);

    // Right wheel must have same velocity magnitude as left wheel, but opposite sign.
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[RIGHT_WHEEL_SPACE_INDEX],
                     -calculated_wheel_speeds[LEFT_WHEEL_SPACE_INDEX]);
}

TEST_F(EuclideanToThreeWheelTest, test_target_wheel_speeds_positive_w)
{
    // Test +w/counter-clockwise
    target_euclidean_velocity = {0, 0, 1};
    calculated_wheel_speeds =
        euclidean_to_three_wheel.getWheelVelocity(target_euclidean_velocity);

    // Formula for the length of a segment: length = radius * angle
    // Since angle = 1rad, the length of the segment is equal to the radius.
    // Therefore, all wheel speeds must be equal to the robot radius.
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[FRONT_WHEEL_SPACE_INDEX],
                     robot_radius);
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[LEFT_WHEEL_SPACE_INDEX], robot_radius);
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[RIGHT_WHEEL_SPACE_INDEX], robot_radius);
}

TEST_F(EuclideanToThreeWheelTest, test_target_wheel_speeds_negative_w)
{
    // Test -w/clockwise
    target_euclidean_velocity = {0, 0, -1};
    calculated_wheel_speeds =
        euclidean_to_three_wheel.getWheelVelocity(target_euclidean_velocity);

    // Formula for the length of a segment: length = radius * angle
    // Since angle = -1rad, the length of the segment is equal to the -radius.
    // Therefore, all wheel speeds (=length of segment/sec) must be equal to the robot
    // radius.
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[FRONT_WHEEL_SPACE_INDEX],
                     -robot_radius);
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[LEFT_WHEEL_SPACE_INDEX],
                     -robot_radius);
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[RIGHT_WHEEL_SPACE_INDEX], -robot_radius);
}

TEST_F(EuclideanToThreeWheelTest, test_conversion_is_linear)
{
    target_euclidean_velocity = {3, 1, 5};
    auto result = euclidean_to_three_wheel.getWheelVelocity(target_euclidean_velocity);

    target_euclidean_velocity = {300, 100, 500};
    auto scaled_result =
        euclidean_to_three_wheel.getWheelVelocity(target_euclidean_velocity);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(result * 100, scaled_result, 0.001));
}

TEST_F(EuclideanToThreeWheelTest, test_double_convertion)
{
    // Converting from euclidean to wheel velocity and back should result in the same
    // value
    target_euclidean_velocity = {3, 1, 5};

    auto wheel_velocity =
        euclidean_to_three_wheel.getWheelVelocity(target_euclidean_velocity);
    auto calculated_euclidean_velocity =
        euclidean_to_three_wheel.getEuclideanVelocity(wheel_velocity);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(target_euclidean_velocity,
                                               calculated_euclidean_velocity, 0.001));
}

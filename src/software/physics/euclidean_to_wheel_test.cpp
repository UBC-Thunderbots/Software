#include "software/physics/euclidean_to_wheel.h"

#include <gtest/gtest.h>

#include "software/test_util/test_util.h"

class EuclideanToWheelTest : public ::testing::Test
{
   protected:
    EuclideanToWheelTest() = default;
    EuclideanSpace_t target_euclidean_velocity{};
    WheelSpace_t expected_wheel_speeds{};
    WheelSpace_t calculated_wheel_speeds{};
    RobotConstants robot_constants = create2021RobotConstants();
    double robot_radius = create2021RobotConstants().robot_radius_m;

    WheelSpace_t current_wheel_velocity{};
    std::pair<Vector, AngularVelocity> current_velocity{};
    TbotsProto::DirectControlPrimitive target_velocity_primitive{};
    double time_to_ramp{};



    EuclideanToWheel euclidean_to_four_wheel =
        EuclideanToWheel(create2021RobotConstants());
};

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_zero)
{
    target_euclidean_velocity = {0, 0, 0};
    expected_wheel_speeds     = {0, 0, 0, 0};

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
        expected_wheel_speeds,
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity), 0.001));
}

// Note: The tests below assume that counter-clockwise motor rotation is positive
// velocity, and vise-versa.
TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_positive_x)
{
    // Test +x/right
    target_euclidean_velocity = {1, 0, 0};
    calculated_wheel_speeds =
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity);

    // Front wheels must be - velocity, back wheels must be + velocity.
    EXPECT_LT(calculated_wheel_speeds[FRONT_RIGHT_WHEEL_SPACE_INDEX], 0);
    EXPECT_LT(calculated_wheel_speeds[FRONT_LEFT_WHEEL_SPACE_INDEX], 0);
    EXPECT_GT(calculated_wheel_speeds[BACK_LEFT_WHEEL_SPACE_INDEX], 0);
    EXPECT_GT(calculated_wheel_speeds[BACK_RIGHT_WHEEL_SPACE_INDEX], 0);
}

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_negative_x)
{
    // Test -x/left
    target_euclidean_velocity = {-1, 0, 0};
    calculated_wheel_speeds =
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity);

    // Front wheels must be + velocity, back wheels must be - velocity.
    EXPECT_GT(calculated_wheel_speeds[FRONT_RIGHT_WHEEL_SPACE_INDEX], 0);
    EXPECT_GT(calculated_wheel_speeds[FRONT_LEFT_WHEEL_SPACE_INDEX], 0);
    EXPECT_LT(calculated_wheel_speeds[BACK_LEFT_WHEEL_SPACE_INDEX], 0);
    EXPECT_LT(calculated_wheel_speeds[BACK_RIGHT_WHEEL_SPACE_INDEX], 0);
}

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_positive_y)
{
    // Test +y/forwards
    target_euclidean_velocity = {0, 1, 0};
    calculated_wheel_speeds =
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity);

    // Right wheels must be + velocity, Left wheels must be - velocity.
    EXPECT_GT(calculated_wheel_speeds[FRONT_RIGHT_WHEEL_SPACE_INDEX], 0);
    EXPECT_LT(calculated_wheel_speeds[FRONT_LEFT_WHEEL_SPACE_INDEX], 0);
    EXPECT_LT(calculated_wheel_speeds[BACK_LEFT_WHEEL_SPACE_INDEX], 0);
    EXPECT_GT(calculated_wheel_speeds[BACK_RIGHT_WHEEL_SPACE_INDEX], 0);

    // Right wheels must have same velocity magnitude as left wheels, but opposite sign.
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[FRONT_RIGHT_WHEEL_SPACE_INDEX],
                     -calculated_wheel_speeds[FRONT_LEFT_WHEEL_SPACE_INDEX]);
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[BACK_LEFT_WHEEL_SPACE_INDEX],
                     -calculated_wheel_speeds[BACK_RIGHT_WHEEL_SPACE_INDEX]);
}

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_negative_y)
{
    // Test -y/backwards
    target_euclidean_velocity = {0, -1, 0};
    calculated_wheel_speeds =
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity);

    // Right wheels must be + velocity, Left wheels must be - velocity.
    EXPECT_LT(calculated_wheel_speeds[FRONT_RIGHT_WHEEL_SPACE_INDEX], 0);
    EXPECT_GT(calculated_wheel_speeds[FRONT_LEFT_WHEEL_SPACE_INDEX], 0);
    EXPECT_GT(calculated_wheel_speeds[BACK_LEFT_WHEEL_SPACE_INDEX], 0);
    EXPECT_LT(calculated_wheel_speeds[BACK_RIGHT_WHEEL_SPACE_INDEX], 0);

    // Right wheels must have same velocity magnitude as left wheels, but opposite sign.
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[FRONT_RIGHT_WHEEL_SPACE_INDEX],
                     -calculated_wheel_speeds[FRONT_LEFT_WHEEL_SPACE_INDEX]);
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[BACK_LEFT_WHEEL_SPACE_INDEX],
                     -calculated_wheel_speeds[BACK_RIGHT_WHEEL_SPACE_INDEX]);
}

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_positive_w)
{
    // Test +w/counter-clockwise
    target_euclidean_velocity = {0, 0, 1};
    calculated_wheel_speeds =
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity);

    // Formula for the length of a segment: length = radius * angle
    // Since angle = 1rad, the length of the segment is equal to the radius.
    // Therefore, all wheel speeds must be equal to the robot radius.
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[FRONT_RIGHT_WHEEL_SPACE_INDEX],
                     robot_radius);
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[FRONT_LEFT_WHEEL_SPACE_INDEX], robot_radius);
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[BACK_LEFT_WHEEL_SPACE_INDEX], robot_radius);
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[BACK_RIGHT_WHEEL_SPACE_INDEX], robot_radius);
}

TEST_F(EuclideanToWheelTest, test_target_wheel_speeds_negative_w)
{
    // Test -w/clockwise
    target_euclidean_velocity = {0, 0, -1};
    calculated_wheel_speeds =
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity);

    // Formula for the length of a segment: length = radius * angle
    // Since angle = -1rad, the length of the segment is equal to the -radius.
    // Therefore, all wheel speeds (=length of segment/sec) must be equal to the robot
    // radius.
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[FRONT_RIGHT_WHEEL_SPACE_INDEX],
                     -robot_radius);
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[FRONT_LEFT_WHEEL_SPACE_INDEX],
                     -robot_radius);
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[BACK_LEFT_WHEEL_SPACE_INDEX], -robot_radius);
    EXPECT_DOUBLE_EQ(calculated_wheel_speeds[BACK_RIGHT_WHEEL_SPACE_INDEX],
                     -robot_radius);
}

TEST_F(EuclideanToWheelTest, test_conversion_is_linear)
{
    target_euclidean_velocity = {3, 1, 5};
    auto result = euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity);

    target_euclidean_velocity = {300, 100, 500};
    auto scaled_result =
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(result * 100, scaled_result, 0.001));
}

TEST_F(EuclideanToWheelTest, test_double_convertion)
{
    // Converting from euclidean to wheel velocity and back should result in the same
    // value
    target_euclidean_velocity = {3, 1, 5};

    auto wheel_velocity =
        euclidean_to_four_wheel.getWheelVelocity(target_euclidean_velocity);
    auto calculated_euclidean_velocity =
        euclidean_to_four_wheel.getEuclideanVelocity(wheel_velocity);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(target_euclidean_velocity,
                                               calculated_euclidean_velocity, 0.001));
}

TEST_F(EuclideanToWheelTest, test_velocity_no_ramping)
{
    time_to_ramp = 0.1;
    auto allowable_delta_wheel_velocity = static_cast<double>(robot_constants.robot_max_acceleration_m_per_s_2) * time_to_ramp;
    auto max_allowable_wheel_velocity = static_cast<double>(robot_constants.robot_max_speed_m_per_s);

    current_wheel_velocity = euclidean_to_four_wheel.getWheelVelocity({10, 10, 10});

    auto current_wheel_velocity_coefficient =
            std::min(max_allowable_wheel_velocity * 0.9, allowable_delta_wheel_velocity * 0.9);

    target_euclidean_velocity = {
            current_wheel_velocity_coefficient,
            current_wheel_velocity_coefficient,
            current_wheel_velocity_coefficient
    };

    auto ramped_wheel_velocity = euclidean_to_four_wheel.rampWheelVelocity(
            current_wheel_velocity,
            target_euclidean_velocity,
            0.1);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
            euclidean_to_four_wheel.getEuclideanVelocity(ramped_wheel_velocity),
            target_euclidean_velocity,
            0.001));
}

TEST_F(EuclideanToWheelTest, test_velocity_equal_to_max)
{
    time_to_ramp = 0.1;
    auto allowable_delta_wheel_velocity = static_cast<double>(robot_constants.robot_max_acceleration_m_per_s_2) * time_to_ramp;
    auto max_allowable_wheel_velocity = static_cast<double>(robot_constants.robot_max_speed_m_per_s);

    current_wheel_velocity =  euclidean_to_four_wheel.getWheelVelocity({10, 10, 10});

    auto current_wheel_velocity_coefficient =
            std::min(max_allowable_wheel_velocity, allowable_delta_wheel_velocity);

    target_euclidean_velocity = {
            current_wheel_velocity_coefficient,
            current_wheel_velocity_coefficient,
            current_wheel_velocity_coefficient
    };

    auto ramped_wheel_velocity = euclidean_to_four_wheel.rampWheelVelocity(
            current_wheel_velocity,
            target_euclidean_velocity,
            0.1);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
            euclidean_to_four_wheel.getEuclideanVelocity(ramped_wheel_velocity),
            target_euclidean_velocity,
            0.001));
}

TEST_F(EuclideanToWheelTest, test_velocity_first_ramp)
{
    time_to_ramp = 0.1;
    auto allowable_delta_wheel_velocity = static_cast<double>(robot_constants.robot_max_acceleration_m_per_s_2) * time_to_ramp;

    current_wheel_velocity =  euclidean_to_four_wheel.getWheelVelocity({10, 10, 10});

    auto current_wheel_velocity_coefficient = 1.01 * allowable_delta_wheel_velocity;

    target_euclidean_velocity = {
            current_wheel_velocity_coefficient,
            current_wheel_velocity_coefficient,
            current_wheel_velocity_coefficient
    };

    auto ramped_wheel_velocity = euclidean_to_four_wheel.rampWheelVelocity(
            current_wheel_velocity,
            target_euclidean_velocity,
            0.1);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
            euclidean_to_four_wheel.getEuclideanVelocity(ramped_wheel_velocity),
            {
                allowable_delta_wheel_velocity,
                allowable_delta_wheel_velocity,
                allowable_delta_wheel_velocity
            }, 0.001));
}

TEST_F(EuclideanToWheelTest, test_velocity_second_ramp)
{
    time_to_ramp = 0.1;
    auto allowable_delta_wheel_velocity = static_cast<double>(robot_constants.robot_max_acceleration_m_per_s_2) * time_to_ramp;
    auto max_allowable_wheel_velocity = static_cast<double>(robot_constants.robot_max_speed_m_per_s);

    current_wheel_velocity = euclidean_to_four_wheel.getWheelVelocity({10, 10, 10});

    auto current_wheel_velocity_coefficient = 0.9 * allowable_delta_wheel_velocity;

    target_euclidean_velocity = {
            10 + current_wheel_velocity_coefficient,
            10 + current_wheel_velocity_coefficient,
            10 + current_wheel_velocity_coefficient
    };

    auto ramped_wheel_velocity = euclidean_to_four_wheel.rampWheelVelocity(
            current_wheel_velocity,
            target_euclidean_velocity,
            0.1);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
            euclidean_to_four_wheel.getEuclideanVelocity(ramped_wheel_velocity),
            {
                max_allowable_wheel_velocity,
                max_allowable_wheel_velocity,
                max_allowable_wheel_velocity
            }, 0.001));
}

TEST_F(EuclideanToWheelTest, test_velocity_both_ramps)
{
    time_to_ramp = 0.1;
    auto allowable_delta_wheel_velocity = static_cast<double>(robot_constants.robot_max_acceleration_m_per_s_2) * time_to_ramp;
    auto max_allowable_wheel_velocity = static_cast<double>(robot_constants.robot_max_speed_m_per_s);

    current_wheel_velocity = euclidean_to_four_wheel.getWheelVelocity({10, 10, 10});

    auto current_wheel_velocity_coefficient = 1.1 * allowable_delta_wheel_velocity;

    target_euclidean_velocity = {
            10 + current_wheel_velocity_coefficient,
            10 + current_wheel_velocity_coefficient,
            10 + current_wheel_velocity_coefficient
    };

    auto ramped_wheel_velocity = euclidean_to_four_wheel.rampWheelVelocity(
            current_wheel_velocity,
            target_euclidean_velocity,
            0.1);

    EXPECT_TRUE(TestUtil::equalWithinTolerance(
            euclidean_to_four_wheel.getEuclideanVelocity(ramped_wheel_velocity),
            {
                max_allowable_wheel_velocity,
                max_allowable_wheel_velocity,
                max_allowable_wheel_velocity
            }, 0.001));
}





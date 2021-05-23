extern "C"
{
#include "firmware/shared/physics.h"
}

#include <gtest/gtest.h>
#include <math.h>

#include "software/test_util/equal_within_tolerance.h"

class PhysicsTestSpeed3Speed4Conversions
    : public ::testing::TestWithParam<std::tuple<float, float, float>>
{
};

TEST_P(PhysicsTestSpeed3Speed4Conversions, test_speed_3_speed_4_conversions_recovered)
{
    const float original_robot_velocity[3] = {
        std::get<0>(GetParam()), std::get<1>(GetParam()), std::get<2>(GetParam())};
    float wheel_velocity[4];
    speed3_to_speed4(original_robot_velocity, wheel_velocity);
    float recovered_robot_velocity[3];
    speed4_to_speed3(wheel_velocity, recovered_robot_velocity);
    for (unsigned int i = 0; i < 3; i++)
    {
        EXPECT_TRUE(TestUtil::equalWithinTolerance(original_robot_velocity[i],
                                                   recovered_robot_velocity[i], 1e-3));
    }
}

INSTANTIATE_TEST_CASE_P(
    velocities, PhysicsTestSpeed3Speed4Conversions,
    ::testing::Values(
        // +/- x
        std::make_tuple(2.0f, 0.0f, 0.0f), std::make_tuple(-2.5f, 0.0f, 0.0f),
        // +/- y
        std::make_tuple(0.0f, 3.0f, 0.0f), std::make_tuple(0.0f, -1.0f, 0.0f),
        // +/- theta
        std::make_tuple(0.0f, 0.0f, 1.0f), std::make_tuple(0.0f, 0.0f, -4.0f),
        // +/- x/y
        std::make_tuple(2.0f, 1.0f, 0.0f), std::make_tuple(-1.0f, -3.0f, 0.0f),
        std::make_tuple(2.0f, -1.0f, 0.0f), std::make_tuple(1.0f, -3.0f, 0.0f),
        // +/- x/theta
        std::make_tuple(3.0f, 0.0f, 2.0f), std::make_tuple(-2.5f, 0.0f, -1.0f),
        // +/- y/theta
        std::make_tuple(0.0f, -3.0f, 2.0f), std::make_tuple(0.0f, 3.0f, -1.0f),
        // +/- x/y/theta
        std::make_tuple(2.0f, -1.0f, -2.0f), std::make_tuple(-1.0f, 3.0f, -2.0f),
        std::make_tuple(1.0f, -3.0f, 2.0f)));

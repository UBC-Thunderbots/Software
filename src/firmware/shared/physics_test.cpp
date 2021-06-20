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
    shared_physics_speed3ToSpeed4(original_robot_velocity, wheel_velocity, 55, 45);
    float recovered_robot_velocity[3];
    shared_physics_speed4ToSpeed3(wheel_velocity, recovered_robot_velocity, 55, 45);
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

class PhysicsTestSpeed4Speed3Conversions
    : public ::testing::TestWithParam<std::tuple<float, float, float, float>>
{
};

TEST_P(PhysicsTestSpeed4Speed3Conversions, test_speed_3_speed_4_conversions_recovered)
{
    // converts speed4 to speed 3 before checking if shared_physics_speed3ToSpeed4 and
    // back is recovered recovering speed4 after shared_physics_speed4ToSpeed3 and
    // shared_physics_speed3ToSpeed4 doesn't work because there are many possible
    // solutions to shared_physics_speed3ToSpeed4
    const float original_wheel_velocity[4] = {
        std::get<0>(GetParam()), std::get<1>(GetParam()), std::get<2>(GetParam()),
        std::get<3>(GetParam())};
    float original_robot_velocity[3];
    shared_physics_speed4ToSpeed3(original_wheel_velocity, original_robot_velocity, 55,
                                  45);

    float wheel_velocity[4];
    shared_physics_speed3ToSpeed4(original_robot_velocity, wheel_velocity, 55, 45);
    float recovered_robot_velocity[3];
    shared_physics_speed4ToSpeed3(wheel_velocity, recovered_robot_velocity, 55, 45);
    for (unsigned int i = 0; i < 3; i++)
    {
        EXPECT_TRUE(TestUtil::equalWithinTolerance(original_robot_velocity[i],
                                                   recovered_robot_velocity[i], 1));
    }
}

INSTANTIATE_TEST_CASE_P(velocities, PhysicsTestSpeed4Speed3Conversions,
                        ::testing::Values(
                            // wheel 0
                            std::make_tuple(200.0f, 0.0f, 0.0f, 0.0f),
                            std::make_tuple(-300.0f, 0.0f, 0.0f, 0.0f),
                            // wheel 1
                            std::make_tuple(0.0f, 400.0f, 0.0f, 0.0f),
                            std::make_tuple(0.0f, -500.0f, 0.0f, 0.0f),
                            // wheel 2
                            std::make_tuple(0.0f, 0.0f, 100.0f, 0.0f),
                            std::make_tuple(0.0f, 0.0f, -600.0f, 0.0f),
                            // wheel 3
                            std::make_tuple(0.0f, 0.0f, 0.0f, 340.0f),
                            std::make_tuple(0.0f, 0.0f, 0.0f, -500.0f),
                            // wheel 0/1
                            std::make_tuple(200.0f, 230.0f, 0.0f, 0.0f),
                            std::make_tuple(-300.0f, -110.0f, 0.0f, 0.0f),
                            std::make_tuple(200.0f, -230.0f, 0.0f, 0.0f),
                            std::make_tuple(-110.0f, 240.0f, 0.0f, 0.0f),
                            // wheel 2/3
                            std::make_tuple(0.0f, 0.0f, 200.0f, 230.0f),
                            std::make_tuple(0.0f, 0.0f, -300.0f, -110.0f),
                            std::make_tuple(0.0f, 0.0f, 200.0f, -230.0f),
                            std::make_tuple(0.0f, 0.0f, -110.0f, 240.0f),
                            // all wheels
                            std::make_tuple(100.0f, -320.0f, 200.0f, 230.0f),
                            std::make_tuple(280.0f, -620.0f, -300.0f, -110.0f),
                            std::make_tuple(-100.0f, 320.0f, 200.0f, -230.0f),
                            std::make_tuple(280.0f, -620.0f, -110.0f, 240.0f)));

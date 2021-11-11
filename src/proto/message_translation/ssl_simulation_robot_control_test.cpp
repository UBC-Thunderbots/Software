#include "proto/message_translation/ssl_simulation_robot_control.h"

#include <gtest/gtest.h>

#include "shared/2015_robot_constants.h"

class SSLSimulationProtoTest : public ::testing::Test
{
   protected:
    WheelConstants wheel_constants = create2015WheelConstants();
};

TEST_F(SSLSimulationProtoTest, test_create_robot_move_command_stationary)
{
    auto move_command =
        createRobotMoveCommand(0, 0, 0, 0, 55, 45, wheel_constants.wheel_radius_meters);
    ASSERT_TRUE(move_command);

    // Expect that the local velocity has some positive value, and that there is minimum
    // left or angular velocity
    EXPECT_NEAR(move_command->local_velocity().forward(), 0.0, 1e-5);
    EXPECT_NEAR(move_command->local_velocity().left(), 0.0, 1e-5);
    EXPECT_NEAR(move_command->local_velocity().angular(), 0.0, 1e-5);
}

TEST_F(SSLSimulationProtoTest, test_create_robot_move_command_forward)
{
    auto move_command = createRobotMoveCommand(60, -60, -60, 60, 55, 45,
                                               wheel_constants.wheel_radius_meters);
    ASSERT_TRUE(move_command);

    // Expect that the local velocity has some positive value, and that there is minimum
    // left or angular velocity
    EXPECT_GT(move_command->local_velocity().forward(), 0.1);
    EXPECT_NEAR(move_command->local_velocity().left(), 0.0, 1e-5);
    EXPECT_NEAR(move_command->local_velocity().angular(), 0.0, 1e-5);
}

TEST_F(SSLSimulationProtoTest, test_create_robot_move_command_forward_from_primitive)
{
    TbotsProto::DirectControlPrimitive test;

    test.mutable_direct_velocity_control()->mutable_velocity()->set_x_component_meters(
        10);
    test.mutable_direct_velocity_control()->mutable_velocity()->set_y_component_meters(5);
    test.mutable_direct_velocity_control()
        ->mutable_angular_velocity()
        ->set_radians_per_second(2);

    auto move_command =
        createRobotMoveCommand(test, 35, 45, wheel_constants.wheel_radius_meters);

    EXPECT_EQ(move_command->local_velocity().left(), 5);
    EXPECT_EQ(move_command->local_velocity().forward(), 10);
    EXPECT_EQ(move_command->local_velocity().angular(), 2);
}

TEST_F(SSLSimulationProtoTest, test_create_robot_move_command_backward)
{
    auto move_command = createRobotMoveCommand(-60, 60, 60, -60, 55, 45,
                                               wheel_constants.wheel_radius_meters);
    ASSERT_TRUE(move_command);

    // Expect that the local velocity has some positive value, and that there is minimum
    // left or angular velocity
    EXPECT_LT(move_command->local_velocity().forward(), -0.1);
    EXPECT_NEAR(move_command->local_velocity().left(), 0.0, 1e-5);
    EXPECT_NEAR(move_command->local_velocity().angular(), 0.0, 1e-5);
}

TEST_F(SSLSimulationProtoTest, test_create_robot_command)
{
    auto move_command  = createRobotMoveCommand(60, -60, -60, 60, 55, 45,
                                               wheel_constants.wheel_radius_meters);
    auto robot_command = createRobotCommand(1, std::move(move_command), 2.0, 3.0, 4.0);

    ASSERT_TRUE(robot_command);
    ASSERT_TRUE(robot_command->has_move_command());

    EXPECT_EQ(1, robot_command->id());
    EXPECT_FLOAT_EQ(2.0, robot_command->kick_speed());
    EXPECT_FLOAT_EQ(3.0, robot_command->kick_angle());
    EXPECT_FLOAT_EQ(4.0, robot_command->dribbler_speed());

    EXPECT_GT(robot_command->move_command().local_velocity().forward(), 0.1);
    EXPECT_NEAR(robot_command->move_command().local_velocity().left(), 0.0, 1e-5);
    EXPECT_NEAR(robot_command->move_command().local_velocity().angular(), 0.0, 1e-5);
}

TEST_F(SSLSimulationProtoTest, test_create_robot_command_unset_optional_fields)
{
    auto move_command  = createRobotMoveCommand(60, -60, -60, 60, 55, 45,
                                               wheel_constants.wheel_radius_meters);
    auto robot_command = createRobotCommand(1, std::move(move_command), std::nullopt,
                                            std::nullopt, std::nullopt);

    ASSERT_TRUE(robot_command);
    ASSERT_TRUE(robot_command->has_move_command());

    EXPECT_EQ(1, robot_command->id());
    EXPECT_FALSE(robot_command->has_kick_speed());
    EXPECT_FALSE(robot_command->has_kick_angle());
    EXPECT_FALSE(robot_command->has_dribbler_speed());

    EXPECT_GT(robot_command->move_command().local_velocity().forward(), 0.1);
    EXPECT_NEAR(robot_command->move_command().local_velocity().left(), 0.0, 1e-5);
    EXPECT_NEAR(robot_command->move_command().local_velocity().angular(), 0.0, 1e-5);
    ;
}

TEST_F(SSLSimulationProtoTest, test_create_robot_control)
{
    auto move_command_1 = createRobotMoveCommand(60, -60, -60, 60, 55, 45,
                                                 wheel_constants.wheel_radius_meters);
    auto robot_command_1 =
        createRobotCommand(1, std::move(move_command_1), 2.0, 3.0, 4.0);

    auto move_command_2 = createRobotMoveCommand(-60, 60, 60, -60, 55, 45,
                                                 wheel_constants.wheel_radius_meters);
    auto robot_command_2 =
        createRobotCommand(2, std::move(move_command_2), 4.0, 6.0, 8.0);

    std::vector<std::unique_ptr<SSLSimulationProto::RobotCommand>> robot_commands = {};
    robot_commands.push_back(std::move(robot_command_1));
    robot_commands.push_back(std::move(robot_command_2));

    auto robot_control = createRobotControl(std::move(robot_commands));

    unsigned robot_id = 1;
    for (auto robot_command : *robot_control->mutable_robot_commands())
    {
        // Negative sign if robot id is even
        float sign = robot_id % 2 == 0 ? -1.0f : 1.0f;
        EXPECT_EQ(robot_id, robot_command.id());
        EXPECT_GT(sign * robot_command.move_command().local_velocity().forward(), 0.1);
        EXPECT_NEAR(robot_command.move_command().local_velocity().left(), 0.0, 1e-5);
        EXPECT_NEAR(robot_command.move_command().local_velocity().angular(), 0.0, 1e-5);
        robot_id++;
    }
}

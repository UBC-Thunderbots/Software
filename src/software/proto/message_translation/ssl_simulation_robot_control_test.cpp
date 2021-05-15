#include "software/proto/message_translation/ssl_simulation_robot_control.h"

#include <gtest/gtest.h>

TEST(SSLSimulationProtoTest, test_create_move_wheel_velocity)
{
    auto move_wheel_velocity = createMoveWheelVelocity(1.0, 2.0, 3.0, 4.0);
    ASSERT_TRUE(move_wheel_velocity);

    EXPECT_FLOAT_EQ(1.0, move_wheel_velocity->front_right());
    EXPECT_FLOAT_EQ(2.0, move_wheel_velocity->front_left());
    EXPECT_FLOAT_EQ(3.0, move_wheel_velocity->back_left());
    EXPECT_FLOAT_EQ(4.0, move_wheel_velocity->back_right());
}

TEST(SSLSimulationProtoTest, test_create_move_local_velocity_stationary)
{
    auto move_local_velocity = createMoveLocalVelocity(0, 0, 0, 0);
    ASSERT_TRUE(move_local_velocity);

    // Expect that the local velocity has some positive value, and that there is minimum
    // left or angular velocity
    EXPECT_NEAR(move_local_velocity->forward(), 0.0, 1e-5);
    EXPECT_NEAR(move_local_velocity->left(), 0.0, 1e-5);
    EXPECT_NEAR(move_local_velocity->angular(), 0.0, 1e-5);
}

TEST(SSLSimulationProtoTest, test_create_move_local_velocity_forward)
{
    auto move_local_velocity = createMoveLocalVelocity(60, -60, -60, 60);
    ASSERT_TRUE(move_local_velocity);

    // Expect that the local velocity has some positive value, and that there is minimum
    // left or angular velocity
    EXPECT_GT(move_local_velocity->forward(), 0.1);
    EXPECT_NEAR(move_local_velocity->left(), 0.0, 1e-5);
    EXPECT_NEAR(move_local_velocity->angular(), 0.0, 1e-5);
}

TEST(SSLSimulationProtoTest, test_create_move_local_velocity_backward)
{
    auto move_local_velocity = createMoveLocalVelocity(-60, 60, 60, -60);
    ASSERT_TRUE(move_local_velocity);

    // Expect that the local velocity has some positive value, and that there is minimum
    // left or angular velocity
    EXPECT_LT(move_local_velocity->forward(), -0.1);
    EXPECT_NEAR(move_local_velocity->left(), 0.0, 1e-5);
    EXPECT_NEAR(move_local_velocity->angular(), 0.0, 1e-5);
}

TEST(SSLSimulationProtoTest, test_create_robot_wheel_velocity_move_command)
{
    auto move_wheel_velocity = createMoveWheelVelocity(1.0, 2.0, 3.0, 4.0);
    auto move_command        = createRobotMoveCommand(std::move(move_wheel_velocity));

    ASSERT_TRUE(move_command);
    ASSERT_TRUE(move_command->has_wheel_velocity());

    EXPECT_FLOAT_EQ(1.0, move_command->wheel_velocity().front_right());
    EXPECT_FLOAT_EQ(2.0, move_command->wheel_velocity().front_left());
    EXPECT_FLOAT_EQ(3.0, move_command->wheel_velocity().back_left());
    EXPECT_FLOAT_EQ(4.0, move_command->wheel_velocity().back_right());
}

TEST(SSLSimulationProtoTest, test_create_robot_command)
{
    auto move_wheel_velocity = createMoveWheelVelocity(1.0, 2.0, 3.0, 4.0);
    auto move_command        = createRobotMoveCommand(std::move(move_wheel_velocity));
    auto robot_command = createRobotCommand(1, std::move(move_command), 2.0, 3.0, 4.0);

    ASSERT_TRUE(robot_command);
    ASSERT_TRUE(robot_command->has_move_command());

    EXPECT_EQ(1, robot_command->id());
    EXPECT_FLOAT_EQ(2.0, robot_command->kick_speed());
    EXPECT_FLOAT_EQ(3.0, robot_command->kick_angle());
    EXPECT_FLOAT_EQ(4.0, robot_command->dribbler_speed());

    EXPECT_FLOAT_EQ(1.0, robot_command->move_command().wheel_velocity().front_right());
    EXPECT_FLOAT_EQ(2.0, robot_command->move_command().wheel_velocity().front_left());
    EXPECT_FLOAT_EQ(3.0, robot_command->move_command().wheel_velocity().back_left());
    EXPECT_FLOAT_EQ(4.0, robot_command->move_command().wheel_velocity().back_right());
}

TEST(SSLSimulationProtoTest, test_create_robot_command_unset_optional_fields)
{
    auto move_wheel_velocity = createMoveWheelVelocity(1.0, 2.0, 3.0, 4.0);
    auto move_command        = createRobotMoveCommand(std::move(move_wheel_velocity));
    auto robot_command = createRobotCommand(1, std::move(move_command), std::nullopt,
                                            std::nullopt, std::nullopt);

    ASSERT_TRUE(robot_command);
    ASSERT_TRUE(robot_command->has_move_command());

    EXPECT_EQ(1, robot_command->id());
    EXPECT_FALSE(robot_command->has_kick_speed());
    EXPECT_FALSE(robot_command->has_kick_angle());
    EXPECT_FALSE(robot_command->has_dribbler_speed());

    EXPECT_FLOAT_EQ(1.0, robot_command->move_command().wheel_velocity().front_right());
    EXPECT_FLOAT_EQ(2.0, robot_command->move_command().wheel_velocity().front_left());
    EXPECT_FLOAT_EQ(3.0, robot_command->move_command().wheel_velocity().back_left());
    EXPECT_FLOAT_EQ(4.0, robot_command->move_command().wheel_velocity().back_right());
}

TEST(SSLSimulationProtoTest, test_create_robot_control)
{
    auto move_wheel_velocity_1 = createMoveWheelVelocity(1.0, 2.0, 3.0, 4.0);
    auto move_command_1        = createRobotMoveCommand(std::move(move_wheel_velocity_1));
    auto robot_command_1 =
        createRobotCommand(1, std::move(move_command_1), 2.0, 3.0, 4.0);

    auto move_wheel_velocity_2 = createMoveWheelVelocity(2.0, 4.0, 6.0, 8.0);
    auto move_command_2        = createRobotMoveCommand(std::move(move_wheel_velocity_2));
    auto robot_command_2 =
        createRobotCommand(2, std::move(move_command_2), 4.0, 6.0, 8.0);

    std::vector<std::unique_ptr<SSLSimulationProto::RobotCommand>> robot_commands = {};
    robot_commands.push_back(std::move(robot_command_1));
    robot_commands.push_back(std::move(robot_command_2));

    auto robot_control = createRobotControl(std::move(robot_commands));

    unsigned robot_id = 1;
    for (auto robot_command : *robot_control->mutable_robot_commands())
    {
        EXPECT_EQ(robot_id, robot_command.id());
        EXPECT_FLOAT_EQ(static_cast<float>(robot_id) * 1.0f,
                        robot_command.move_command().wheel_velocity().front_right());
        EXPECT_FLOAT_EQ(static_cast<float>(robot_id) * 2.0f,
                        robot_command.move_command().wheel_velocity().front_left());
        EXPECT_FLOAT_EQ(static_cast<float>(robot_id) * 3.0f,
                        robot_command.move_command().wheel_velocity().back_left());
        EXPECT_FLOAT_EQ(static_cast<float>(robot_id) * 4.0f,
                        robot_command.move_command().wheel_velocity().back_right());
        robot_id++;
    }
}

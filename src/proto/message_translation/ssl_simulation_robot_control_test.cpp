#include "proto/message_translation/ssl_simulation_robot_control.h"

#include <gtest/gtest.h>

#include "shared/2021_robot_constants.h"

class SSLSimulationProtoTest : public ::testing::Test
{
   protected:
    RobotConstants_t robot_constants = create2021RobotConstants();
};

TEST_F(SSLSimulationProtoTest, test_create_robot_move_command_forward_from_primitive)
{
    TbotsProto::DirectControlPrimitive test;

    test.mutable_motor_control()
        ->mutable_direct_velocity_control()
        ->mutable_velocity()
        ->set_x_component_meters(10);
    test.mutable_motor_control()
        ->mutable_direct_velocity_control()
        ->mutable_velocity()
        ->set_y_component_meters(5);
    test.mutable_motor_control()
        ->mutable_direct_velocity_control()
        ->mutable_angular_velocity()
        ->set_radians_per_second(2);

    auto move_command =
        createRobotMoveCommand(test, 35, 45, robot_constants.wheel_radius_meters);

    EXPECT_EQ(move_command->local_velocity().left(), 5);
    EXPECT_EQ(move_command->local_velocity().forward(), 10);
    EXPECT_EQ(move_command->local_velocity().angular(), 2);
}

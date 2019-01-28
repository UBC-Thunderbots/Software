/**
 * This file contains the unit tests for the MoveSpinPrimitive class
 */

#include "ai/primitive/directwheels_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(DirectWheelsPrimTest, construct_with_no_params_test)
{
    const std::string directwheels_prim_name = "DirectWheels Primitive";

    DirectWheelsPrimitive directwheels_prim = DirectWheelsPrimitive(0, 0, 0, 0, 0, 0.0);

    EXPECT_EQ(0, directwheels_prim.getRobotId());
    EXPECT_EQ(directwheels_prim_name, directwheels_prim.getPrimitiveName());
}

TEST(DirectWheelsPrimTest, get_robot_id_test)
{
    unsigned int robot_id = 3U;

    DirectWheelsPrimitive directwheels_prim =
        DirectWheelsPrimitive(robot_id, 0, 0, 0, 0, 0.0);

    EXPECT_EQ(robot_id, directwheels_prim.getRobotId());
}

TEST(DirectWheelsPrimTest, parameter_array_test)
{
    const int wheel0_power      = 1;
    const int wheel1_power      = 2;
    const int wheel2_power      = 3;
    const int wheel3_power      = 4;
    const double dribbler_rpm   = 10.0;
    const unsigned int robot_id = 1U;

    DirectWheelsPrimitive directwheels_prim = DirectWheelsPrimitive(
        robot_id, wheel0_power, wheel1_power, wheel2_power, wheel3_power, dribbler_rpm);

    std::vector<double> param_array = directwheels_prim.getParameters();

    EXPECT_DOUBLE_EQ(wheel0_power, param_array[0]);
    EXPECT_DOUBLE_EQ(wheel1_power, param_array[1]);
    EXPECT_DOUBLE_EQ(wheel2_power, param_array[2]);
    EXPECT_DOUBLE_EQ(wheel3_power, param_array[3]);
    EXPECT_DOUBLE_EQ(dribbler_rpm, param_array[4]);
}

TEST(DirectWheelsPrimTest, get_wheel0_power_test)
{
    const int wheel0_power = 5;

    DirectWheelsPrimitive directwheels_prim =
        DirectWheelsPrimitive(0, wheel0_power, 0, 0, 0, 0.0);

    EXPECT_EQ(directwheels_prim.getWheel0Power(), 5);
}

TEST(DirectWheelsPrimTest, get_wheel1_power_test)
{
    const int wheel1_power = 6;

    DirectWheelsPrimitive directwheels_prim =
        DirectWheelsPrimitive(0, 0, wheel1_power, 0, 0, 0.0);

    EXPECT_EQ(directwheels_prim.getWheel1Power(), 6);
}

TEST(DirectWheelsPrimTest, get_wheel2_power_test)
{
    const int wheel2_power = 7;

    DirectWheelsPrimitive directwheels_prim =
        DirectWheelsPrimitive(0, 0, 0, wheel2_power, 0, 0.0);

    EXPECT_EQ(directwheels_prim.getWheel2Power(), 7);
}

TEST(DirectWheelsPrimTest, get_wheel3_power_test)
{
    const int wheel3_power = 8;

    DirectWheelsPrimitive directwheels_prim =
        DirectWheelsPrimitive(0, 0, 0, 0, wheel3_power, 0.0);

    EXPECT_EQ(directwheels_prim.getWheel3Power(), 8);
}

TEST(DirectWheelsPrimTest, get_dribbler_rpm_test)
{
    const double dribbler_rpm = 99.9;

    DirectWheelsPrimitive directwheels_prim =
        DirectWheelsPrimitive(0, 0, 0, 0, 0, dribbler_rpm);

    EXPECT_EQ(directwheels_prim.getDribblerRPM(), dribbler_rpm);
}

TEST(DirectWheelsPrimTest, get_extra_bit_array_test)
{
    DirectWheelsPrimitive directwheels_prim = DirectWheelsPrimitive(0, 0, 0, 0, 0, 0.0);

    std::vector<bool> extra_bit_array = directwheels_prim.getExtraBits();

    EXPECT_EQ(extra_bit_array, std::vector<bool>());
}

TEST(DirectWheelsPrimTest, create_primitive_from_message_test)
{
    const int wheel0_power      = 1;
    const int wheel1_power      = 2;
    const int wheel2_power      = 3;
    const int wheel3_power      = 4;
    const double dribbler_rpm   = 13.37;
    const unsigned int robot_id = 2U;

    DirectWheelsPrimitive directwheels_prim = DirectWheelsPrimitive(
        robot_id, wheel0_power, wheel1_power, wheel2_power, wheel3_power, dribbler_rpm);

    thunderbots_msgs::Primitive prim_message = directwheels_prim.createMsg();

    DirectWheelsPrimitive new_prim = DirectWheelsPrimitive(prim_message);

    std::vector<double> parameters = new_prim.getParameters();

    EXPECT_EQ("DirectWheels Primitive", new_prim.getPrimitiveName());
    EXPECT_EQ(robot_id, new_prim.getRobotId());
    EXPECT_DOUBLE_EQ(wheel0_power, parameters[0]);
    EXPECT_DOUBLE_EQ(wheel1_power, parameters[1]);
    EXPECT_DOUBLE_EQ(wheel2_power, parameters[2]);
    EXPECT_DOUBLE_EQ(wheel3_power, parameters[3]);
    EXPECT_DOUBLE_EQ(dribbler_rpm, parameters[4]);
    EXPECT_EQ(directwheels_prim.getExtraBits(), std::vector<bool>());
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

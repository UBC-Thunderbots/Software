/**
 * This file contains the unit tests for the MoveSpinPrimitive class
 */

#include "ai/primitive/direct_wheels_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(DirectWheelsPrimTest, primitive_name_test)
{
    DirectWheelsPrimitive directwheels_prim = DirectWheelsPrimitive(0, 0, 0, 0, 0, 0.0);

    EXPECT_EQ("Direct Wheels Primitive", directwheels_prim.getPrimitiveName());
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

    EXPECT_EQ(DirectWheelsPrimitive::PRIMITIVE_NAME, new_prim.getPrimitiveName());
    EXPECT_EQ(new_prim, directwheels_prim);
}

TEST(DirectWheelsPrimTest, test_equality_operator_primitives_equal)
{
    DirectWheelsPrimitive directwheels_primitive =
        DirectWheelsPrimitive(0, 0, 0, 0, 0, 0.0);
    DirectWheelsPrimitive directwheels_primitive_other =
        DirectWheelsPrimitive(0, 0, 0, 0, 0, 0.0);

    EXPECT_EQ(directwheels_primitive, directwheels_primitive_other);
}

TEST(DirectWheelsPrimTest, test_inequality_operator_with_mismatched_robot_id)
{
    DirectWheelsPrimitive directwheels_primitive =
        DirectWheelsPrimitive(0, 0, 0, 0, 0, 0.0);
    DirectWheelsPrimitive directwheels_primitive_other =
        DirectWheelsPrimitive(3, 0, 0, 0, 0, 0.0);

    EXPECT_NE(directwheels_primitive, directwheels_primitive_other);
}

TEST(DirectWheelsPrimTest,
     test_inequality_operator_with_mismatched_front_left_wheel_power)
{
    DirectWheelsPrimitive directwheels_primitive =
        DirectWheelsPrimitive(0, 0, -1, 0, 0, 0.0);
    DirectWheelsPrimitive directwheels_primitive_other =
        DirectWheelsPrimitive(0, 1, 0, 0, 0, 0.0);

    EXPECT_NE(directwheels_primitive, directwheels_primitive_other);
}

TEST(DirectWheelsPrimTest, test_inequality_operator_with_mismatched_back_left_wheel_power)
{
    DirectWheelsPrimitive directwheels_primitive =
        DirectWheelsPrimitive(0, 0, 0, 0, 0, 0.0);
    DirectWheelsPrimitive directwheels_primitive_other =
        DirectWheelsPrimitive(0, 0, -2, 0, 0, 0.0);

    EXPECT_NE(directwheels_primitive, directwheels_primitive_other);
}

TEST(DirectWheelsPrimTest,
     test_inequality_operator_with_mismatched_front_right_wheel_power)
{
    DirectWheelsPrimitive directwheels_primitive =
        DirectWheelsPrimitive(0, 0, 0, 0, 0, 0.0);
    DirectWheelsPrimitive directwheels_primitive_other =
        DirectWheelsPrimitive(0, 0, 0, 240, 0, 0.0);

    EXPECT_NE(directwheels_primitive, directwheels_primitive_other);
}

TEST(DirectWheelsPrimTest,
     test_inequality_operator_with_mismatched_back_right_wheel_power)
{
    DirectWheelsPrimitive directwheels_primitive =
        DirectWheelsPrimitive(0, 0, 0, 0, 0, 0.0);
    DirectWheelsPrimitive directwheels_primitive_other =
        DirectWheelsPrimitive(0, 0, 0, 0, -100.1, 0.0);

    EXPECT_NE(directwheels_primitive, directwheels_primitive_other);
}

TEST(DirectWheelsPrimTest, test_inequality_operator_with_mismatched_dribbler_rpm)
{
    DirectWheelsPrimitive directwheels_primitive =
        DirectWheelsPrimitive(0, 0, 0, 0, 0, 0.0);
    DirectWheelsPrimitive directwheels_primitive_other =
        DirectWheelsPrimitive(0, 0, 0, 0, 0, 999.9);

    EXPECT_NE(directwheels_primitive, directwheels_primitive_other);
}

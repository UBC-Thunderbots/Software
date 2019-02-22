/**
 * This file contains the unit tests for the DirectVelocityPrimitive class
 */

#include "ai/primitive/direct_velocity_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(DirectVelocityPrimTest, primitive_name_test)
{
    DirectVelocityPrimitive direct_velocity_prim =
        DirectVelocityPrimitive(0U, 0.0, 0.0, 0.0, 0.0);

    EXPECT_EQ("Direct Velocity Primitive", direct_velocity_prim.getPrimitiveName());
}

TEST(DirectVelocityPrimTest, get_robot_id_test)
{
    unsigned int robot_id = 4U;

    DirectVelocityPrimitive direct_velocity_prim =
        DirectVelocityPrimitive(robot_id, 0.0, 0.0, 0.0, 0.0);

    EXPECT_EQ(robot_id, direct_velocity_prim.getRobotId());
}

TEST(DirectVelocityPrimTest, parameter_array_test)
{
    const unsigned int robot_id   = 2U;
    const double x_velocity       = 1.25;
    const double y_velocity       = -0.29;
    const double angular_velocity = 0.0;
    const double dribbler_rpm     = 5.5;


    DirectVelocityPrimitive direct_velocity_prim = DirectVelocityPrimitive(
        robot_id, x_velocity, y_velocity, angular_velocity, dribbler_rpm);

    std::vector<double> param_array = direct_velocity_prim.getParameters();

    EXPECT_DOUBLE_EQ(x_velocity, param_array[0]);
    EXPECT_DOUBLE_EQ(y_velocity, param_array[1]);
    EXPECT_DOUBLE_EQ(angular_velocity, param_array[2]);
    EXPECT_DOUBLE_EQ(dribbler_rpm, param_array[3]);
}

TEST(DirectVelocityPrimTest, get_x_velocity_test)
{
    const double x_velocity = 2.11;

    DirectVelocityPrimitive direct_velocity_prim =
        DirectVelocityPrimitive(0, x_velocity, 0.0, 0.0, 0.0);

    EXPECT_DOUBLE_EQ(direct_velocity_prim.getXVelocity(), x_velocity);
}

TEST(DirectVelocityPrimTest, get_y_velocity_test)
{
    const double y_velocity = 2.8;

    DirectVelocityPrimitive direct_velocity_prim =
        DirectVelocityPrimitive(0, 0.0, y_velocity, 0.0, 0.0);

    EXPECT_DOUBLE_EQ(direct_velocity_prim.getYVelocity(), y_velocity);
}

TEST(DirectVelocityPrimTest, get_angular_velocity_test)
{
    const double angular_velocity = -0.4;

    DirectVelocityPrimitive direct_velocity_prim =
        DirectVelocityPrimitive(0, 0.0, 0.0, angular_velocity, 0.0);

    EXPECT_DOUBLE_EQ(direct_velocity_prim.getAngularVelocity(), angular_velocity);
}

TEST(DirectVelocityPrimTest, get_dribbler_rpm_test)
{
    const double dribbler_rpm = -0.4;

    DirectVelocityPrimitive direct_velocity_prim =
        DirectVelocityPrimitive(0, 0.0, 0.0, 0.0, dribbler_rpm);

    EXPECT_DOUBLE_EQ(direct_velocity_prim.getDribblerRpm(), dribbler_rpm);
}


TEST(DirectVelocityPrimTest, get_extra_bit_array_test)
{
    DirectVelocityPrimitive direct_velocity_prim =
        DirectVelocityPrimitive(0, 0.0, 0.0, 0.0, 0.0);

    std::vector<bool> extra_bit_array = direct_velocity_prim.getExtraBits();

    EXPECT_EQ(extra_bit_array, std::vector<bool>());
}

TEST(DirectVelocityPrimTest, create_primitive_from_message_test)
{
    const unsigned int robot_id   = 0U;
    const double x_velocity       = 1.414;
    const double y_velocity       = 9.4;
    const double angular_velocity = -0.09;
    const double dribbler_rpm     = 3.14;

    DirectVelocityPrimitive direct_velocity_prim = DirectVelocityPrimitive(
        robot_id, x_velocity, y_velocity, angular_velocity, dribbler_rpm);

    thunderbots_msgs::Primitive prim_message = direct_velocity_prim.createMsg();

    DirectVelocityPrimitive new_prim = DirectVelocityPrimitive(prim_message);

    std::vector<double> parameters = new_prim.getParameters();

    EXPECT_EQ(DirectVelocityPrimitive::PRIMITIVE_NAME, new_prim.getPrimitiveName());
    EXPECT_EQ(new_prim, direct_velocity_prim);
}

TEST(DirectVelocityPrimTest, test_equality_operator_primitives_equal)
{
    DirectVelocityPrimitive direct_velocity_prim =
        DirectVelocityPrimitive(0, 0.0, 0.0, 0.0, 0.0);
    DirectVelocityPrimitive direct_velocity_prim_other =
        DirectVelocityPrimitive(0, 0.0, 0.0, 0.0, 0.0);

    EXPECT_EQ(direct_velocity_prim, direct_velocity_prim_other);
}

TEST(DirectVelocityPrimTest, test_inequality_operator_with_mismatched_robot_ids)
{
    DirectVelocityPrimitive direct_velocity_prim =
        DirectVelocityPrimitive(0, 0.0, 0.0, 0.0, 0.0);
    DirectVelocityPrimitive direct_velocity_prim_other =
        DirectVelocityPrimitive(6, 0.0, 0.0, 0.0, 0.0);

    EXPECT_NE(direct_velocity_prim, direct_velocity_prim_other);
}

TEST(DirectVelocityPrimTest, test_inequality_operator_with_mismatched_x_velocities)
{
    DirectVelocityPrimitive direct_velocity_prim =
        DirectVelocityPrimitive(0, 0.0, 0.0, 0.0, 0.0);
    DirectVelocityPrimitive direct_velocity_prim_other =
        DirectVelocityPrimitive(0, -0.55, 0.0, 0.0, 0.0);

    EXPECT_NE(direct_velocity_prim, direct_velocity_prim_other);
}

TEST(DirectVelocityPrimTest, test_inequality_operator_with_mismatched_y_velocities)
{
    DirectVelocityPrimitive direct_velocity_prim =
        DirectVelocityPrimitive(0, 0.0, 0.0, 0.0, 0.0);
    DirectVelocityPrimitive direct_velocity_prim_other =
        DirectVelocityPrimitive(0, 0.0, 1.0, 0.0, 0.0);

    EXPECT_NE(direct_velocity_prim, direct_velocity_prim_other);
}

TEST(DirectVelocityPrimTest, test_inequality_operator_with_mismatched_angular_velocities)
{
    DirectVelocityPrimitive direct_velocity_prim =
        DirectVelocityPrimitive(0, 0.0, 0.0, 0.0, 0.0);
    DirectVelocityPrimitive direct_velocity_prim_other =
        DirectVelocityPrimitive(0, 0.0, 0.0, 3.1, 0.0);

    EXPECT_NE(direct_velocity_prim, direct_velocity_prim_other);
}

TEST(DirectVelocityPrimTest, test_inequality_operator_with_mismatched_dribbler_rpm)
{
    DirectVelocityPrimitive direct_velocity_prim =
        DirectVelocityPrimitive(0, 0.0, 0.0, 0.0, 0.0);
    DirectVelocityPrimitive direct_velocity_prim_other =
        DirectVelocityPrimitive(0, 0.0, 0.0, 0.0, 1000);

    EXPECT_NE(direct_velocity_prim, direct_velocity_prim_other);
}

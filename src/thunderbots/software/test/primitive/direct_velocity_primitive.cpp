/**
 * This file contains the unit tests for the DirectVelocityPrimitive class
 */

#include "ai/primitive/direct_velocity_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(DirectVelocityPrimTest, construct_with_no_params_test)
{
    const std::string direct_velocity_prim_name = "Direct Velocity Primitive";

    DirectVelocityPrimitive direct_velocity_prim =
        DirectVelocityPrimitive(0U, 0.0, 0.0, 0.0, 0.0);

    EXPECT_EQ(0U, direct_velocity_prim.getRobotId());
    EXPECT_EQ(direct_velocity_prim_name, direct_velocity_prim.getPrimitiveName());
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

    EXPECT_EQ("Direct Velocity Primitive", new_prim.getPrimitiveName());
    EXPECT_EQ(robot_id, new_prim.getRobotId());
    EXPECT_DOUBLE_EQ(x_velocity, parameters[0]);
    EXPECT_DOUBLE_EQ(y_velocity, parameters[1]);
    EXPECT_DOUBLE_EQ(angular_velocity, parameters[2]);
    EXPECT_DOUBLE_EQ(dribbler_rpm, parameters[3]);
    EXPECT_EQ(new_prim.getExtraBits(), std::vector<bool>());
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

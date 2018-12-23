/**
 * This file contains the unit tests for the DirectVelocityPrimitive class
 */

#include "ai/primitive/directvelocity_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(DirectVelocityPrimTest, construct_with_no_params_test)
{
    const std::string directvel_prim_name = "DirectVelocity Primitive";

    DirectVelocityPrimitive directvel_prim =
        DirectVelocityPrimitive(0, 0.0, 0.0, AngularVelocity(), 0.0);

    EXPECT_EQ(int(), directvel_prim.getRobotId());
    EXPECT_EQ(directvel_prim_name, directvel_prim.getPrimitiveName());
}

TEST(DirectVelocityPrimTest, get_robot_id_test)
{
    unsigned int robot_id = 4U;

    DirectVelocityPrimitive directvel_prim =
        DirectVelocityPrimitive(robot_id, 0.0, 0.0, AngularVelocity(), 0.0);

    EXPECT_EQ(robot_id, directvel_prim.getRobotId());
}

TEST(DirectVelocityPrimTest, parameter_array_test)
{
    const double x_vel                = 1.0;
    const double y_vel                = 2.0;
    const AngularVelocity angular_vel = AngularVelocity::ofRadians(1.25);
    const double dribbler_rpm         = 100.0;
    const unsigned int robot_id       = 2U;

    DirectVelocityPrimitive directvel_prim =
        DirectVelocityPrimitive(robot_id, x_vel, y_vel, angular_vel, dribbler_rpm);

    std::vector<double> param_array = directvel_prim.getParameterArray();

    EXPECT_DOUBLE_EQ(x_vel, param_array[0]);
    EXPECT_DOUBLE_EQ(y_vel, param_array[1]);
    EXPECT_DOUBLE_EQ(angular_vel.toRadians(), param_array[2]);
    EXPECT_DOUBLE_EQ(dribbler_rpm, param_array[3]);
}

TEST(DirectVelocityPrimTest, get_x_vel_test)
{
    const double x_vel = 1.11;

    DirectVelocityPrimitive directvel_prim =
        DirectVelocityPrimitive(0, x_vel, 0.0, AngularVelocity(), 0.0);

    EXPECT_EQ(directvel_prim.getXVelocity(), x_vel);
}

TEST(DirectVelocityPrimTest, get_y_vel_test)
{
    const double y_vel = 2.22;

    DirectVelocityPrimitive directvel_prim =
        DirectVelocityPrimitive(0, 0.0, y_vel, AngularVelocity(), 0.0);

    EXPECT_EQ(directvel_prim.getYVelocity(), y_vel);
}

TEST(DirectVelocityPrimTest, get_angular_vel_test)
{
    const AngularVelocity angular_vel = AngularVelocity::ofRadians(3.33);

    DirectVelocityPrimitive directvel_prim =
        DirectVelocityPrimitive(0, 0.0, 0.0, angular_vel, 0.0);

    EXPECT_EQ(directvel_prim.getAngularVelocity(), angular_vel);
}

TEST(DirectVelocityPrimTest, get_dribbler_rpm_test)
{
    const double dribbler_rpm = 111.1;

    DirectVelocityPrimitive directvel_prim =
        DirectVelocityPrimitive(0, 0.0, 0.0, AngularVelocity(), dribbler_rpm);

    EXPECT_EQ(directvel_prim.getDribblerRPM(), dribbler_rpm);
}

TEST(DirectVelocityPrimTest, get_extra_bit_array_test)
{
    DirectVelocityPrimitive directvel_prim =
        DirectVelocityPrimitive(0, 0.0, 0.0, AngularVelocity(), 0.0);

    std::vector<bool> extra_bit_array = directvel_prim.getExtraBitArray();

    EXPECT_EQ(extra_bit_array, std::vector<bool>());
}

TEST(DirectVelocityPrimTest, create_primitive_from_message_test)
{
    const double x_vel                = 2.7;
    const double y_vel                = 3.7;
    const AngularVelocity angular_vel = AngularVelocity::ofRadians(2.75);
    const double dribbler_rpm         = 42.0;
    const unsigned int robot_id       = 3U;

    DirectVelocityPrimitive directvel_prim =
        DirectVelocityPrimitive(robot_id, x_vel, y_vel, angular_vel, dribbler_rpm);

    thunderbots_msgs::Primitive prim_message = directvel_prim.createMsg();

    DirectVelocityPrimitive new_prim = DirectVelocityPrimitive(prim_message);

    std::vector<double> parameters = new_prim.getParameterArray();

    EXPECT_EQ("DirectVelocity Primitive", new_prim.getPrimitiveName());
    EXPECT_EQ(robot_id, new_prim.getRobotId());
    EXPECT_DOUBLE_EQ(x_vel, parameters[0]);
    EXPECT_DOUBLE_EQ(y_vel, parameters[1]);
    EXPECT_DOUBLE_EQ(angular_vel.toRadians(), parameters[2]);
    EXPECT_DOUBLE_EQ(dribbler_rpm, parameters[3]);
    EXPECT_EQ(directvel_prim.getExtraBitArray(), std::vector<bool>());
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
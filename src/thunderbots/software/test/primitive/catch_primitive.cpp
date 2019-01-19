/**
 * This file contains unit tests for the Catch Primitive class
 */

#include "ai/primitive/catch_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(CatchPrimTest, constuct_with_no_params_test)
{
    const std::string catch_prim_name = "Catch Primitive";

    CatchPrimitive catch_prim = CatchPrimitive(0, 0, 0, 0);

    EXPECT_EQ(int(), catch_prim.getRobotId());
    EXPECT_EQ(catch_prim_name, catch_prim.getPrimitiveName());
}

TEST(CatchPrimTest, get_robot_id_test)
{
    const unsigned int robot_id = 40;

    CatchPrimitive catch_prim = CatchPrimitive(robot_id, 0, 0, 0);

    EXPECT_EQ(robot_id, catch_prim.getRobotId());
}

TEST(CatchPrimTest, parameter_array_test)
{
    const unsigned int robot_id = 14;
    const double velocity       = 3.24;
    const double dribbler_speed = 4.20;
    const double margin         = 17.38;

    CatchPrimitive catch_prim =
        CatchPrimitive(robot_id, velocity, dribbler_speed, margin);

    std::vector<double> param_array = catch_prim.getParameters();

    EXPECT_EQ(velocity, param_array[0]);
    EXPECT_EQ(dribbler_speed, param_array[1]);
    EXPECT_EQ(margin, param_array[2]);
}

TEST(CatchPrimTest, get_velocity_test)
{
    const double velocity = 3.24;

    CatchPrimitive catch_prim = CatchPrimitive(0, velocity, 0, 0);

    EXPECT_DOUBLE_EQ(velocity, catch_prim.getVelocity());
}

TEST(CatchPrimTest, get_dribbler_speed_test)
{
    const double dribbler_speed = 4.20;

    CatchPrimitive catch_prim = CatchPrimitive(0, 0, dribbler_speed, 0);

    EXPECT_EQ(dribbler_speed, catch_prim.getDribblerSpeed());
}

TEST(CatchPrimTest, get_margin_test)
{
    const double margin = 17.38;

    CatchPrimitive catch_prim = CatchPrimitive(0, 0, 0, margin);

    EXPECT_DOUBLE_EQ(margin, catch_prim.getMargin());
}

TEST(CatchPrimTest, get_extra_bit_array_test)
{
    CatchPrimitive catch_prim = CatchPrimitive(0, 0, 0, 0);

    std::vector<bool> extra_bit_array = catch_prim.getExtraBits();

    EXPECT_EQ(extra_bit_array, std::vector<bool>());
}

TEST(CatchPrimTest, create_primitive_from_message_test)
{
    const unsigned int robot_id = 14;
    const double velocity       = 3.24;
    const double dribbler_speed = 4.20;
    const double margin         = 17.38;

    CatchPrimitive catch_prim =
        CatchPrimitive(robot_id, velocity, dribbler_speed, margin);

    thunderbots_msgs::Primitive prim_message = catch_prim.createMsg();

    CatchPrimitive new_prim = CatchPrimitive(prim_message);

    std::vector<double> parameters = new_prim.getParameters();

    EXPECT_EQ("Catch Primitive", new_prim.getPrimitiveName());
    EXPECT_EQ(robot_id, new_prim.getRobotId());
    EXPECT_EQ(velocity, parameters[0]);
    EXPECT_EQ(dribbler_speed, parameters[1]);
    EXPECT_EQ(margin, parameters[2]);
    EXPECT_EQ(catch_prim.getExtraBits(), std::vector<bool>());
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

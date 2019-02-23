/**
 * This file contains unit tests for the Catch Primitive class
 */

#include <gtest/gtest.h>
#include <string.h>

#include "ai/primitive/grsim_command_primitive_visitor_catch.h"

TEST(CatchPrimTest, primitive_name_test)
{
    CatchPrimitive catch_prim = CatchPrimitive(0, 0, 0, 0);

    EXPECT_EQ("Catch Primitive", catch_prim.getPrimitiveName());
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

    EXPECT_EQ(CatchPrimitive::PRIMITIVE_NAME, new_prim.getPrimitiveName());
    EXPECT_EQ(new_prim, catch_prim);
}

TEST(CatchPrimTest, test_equality_operator_primitives_equal)
{
    CatchPrimitive catch_prim       = CatchPrimitive(0, 1, 900, 0.5);
    CatchPrimitive catch_prim_other = CatchPrimitive(0, 1, 900, 0.5);

    EXPECT_EQ(catch_prim, catch_prim_other);
}

TEST(CatchPrimTest, test_inequality_operator_with_mismatched_robot_ids)
{
    CatchPrimitive catch_prim       = CatchPrimitive(0, 1, 900, 0.5);
    CatchPrimitive catch_prim_other = CatchPrimitive(1, 1, 900, 0.5);

    EXPECT_NE(catch_prim, catch_prim_other);
}

TEST(CatchPrimTest, test_inequality_operator_with_mismatched_velocity)
{
    CatchPrimitive catch_prim       = CatchPrimitive(0, -0.5, 900, 0.5);
    CatchPrimitive catch_prim_other = CatchPrimitive(0, 1, 900, 0.5);

    EXPECT_NE(catch_prim, catch_prim_other);
}

TEST(CatchPrimTest, test_inequality_operator_with_mismatched_dribbler_speed)
{
    CatchPrimitive catch_prim       = CatchPrimitive(0, 1, 900, 0.5);
    CatchPrimitive catch_prim_other = CatchPrimitive(0, 1, 901, 0.5);

    EXPECT_NE(catch_prim, catch_prim_other);
}

TEST(CatchPrimTest, test_inequality_operator_with_mismatched_margin)
{
    CatchPrimitive catch_prim       = CatchPrimitive(0, 1, 900, 0.5);
    CatchPrimitive catch_prim_other = CatchPrimitive(0, 1, 900, 0.8);

    EXPECT_NE(catch_prim, catch_prim_other);
}

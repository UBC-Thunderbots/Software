/**
 * This file contains unit tests for the Catch Primitive class
 */

#include "software/primitive/catch_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

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

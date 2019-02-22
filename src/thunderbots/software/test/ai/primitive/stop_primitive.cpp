/**
 * This file contains the unit tests for the StopPrimitive class
 */

#include "ai/primitive/stop_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(StopPrimTest, primitive_name_test)
{
    StopPrimitive stop_prim = StopPrimitive(0, true);

    EXPECT_EQ("Stop Primitive", stop_prim.getPrimitiveName());
}

TEST(StopPrimTest, get_robot_id_test)
{
    unsigned int robot_id = 4U;

    StopPrimitive stop_prim = StopPrimitive(robot_id, false);

    EXPECT_EQ(robot_id, stop_prim.getRobotId());
}

TEST(StopPrimTest, extra_bits_test)
{
    const bool coast = false;

    StopPrimitive stop_prim = StopPrimitive(0, coast);

    std::vector<bool> extra_bits = stop_prim.getExtraBits();

    EXPECT_EQ(coast, extra_bits[0]);
}

TEST(StopPrimTest, paramater_array_test)
{
    const unsigned int robot_id = 2U;
    const bool coast            = false;

    StopPrimitive stop_primitive = StopPrimitive(robot_id, coast);

    std::vector<double> param_array = stop_primitive.getParameters();

    EXPECT_EQ(param_array, std::vector<double>());
}

TEST(StopPrimTest, get_coast_test)
{
    const bool coast = false;

    StopPrimitive stop_prim = StopPrimitive(0, coast);

    EXPECT_EQ(coast, stop_prim.robotShouldCoast());
}

TEST(StopPrimitiveTest, create_primitive_from_message_test)
{
    const unsigned int robot_id = 3U;
    const bool coast            = false;

    StopPrimitive stop_prim = StopPrimitive(robot_id, coast);

    thunderbots_msgs::Primitive prim_message = stop_prim.createMsg();

    StopPrimitive new_prim = StopPrimitive(prim_message);

    std::vector<bool> extra_bits = new_prim.getExtraBits();

    EXPECT_EQ(StopPrimitive::PRIMITIVE_NAME, new_prim.getPrimitiveName());
    EXPECT_EQ(new_prim, stop_prim);
}

TEST(StopPrimitiveTest, test_equality_operator_primitives_equal)
{
    StopPrimitive stop_prim       = StopPrimitive(0, true);
    StopPrimitive stop_prim_other = StopPrimitive(0, true);

    EXPECT_EQ(stop_prim, stop_prim_other);
}

TEST(StopPrimitiveTest, test_inequality_operator_with_mismatched_robot_id)
{
    StopPrimitive stop_prim       = StopPrimitive(0, true);
    StopPrimitive stop_prim_other = StopPrimitive(3, true);

    EXPECT_NE(stop_prim, stop_prim_other);
}

TEST(StopPrimitiveTest, test_inequality_operator_with_mismatched_coast_value)
{
    StopPrimitive stop_prim       = StopPrimitive(0, true);
    StopPrimitive stop_prim_other = StopPrimitive(0, false);

    EXPECT_NE(stop_prim, stop_prim_other);
}

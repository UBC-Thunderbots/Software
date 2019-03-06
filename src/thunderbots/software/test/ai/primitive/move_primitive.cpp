/**
 * This file contains the unit tests for the MovePrimitive class
 */

#include "ai/primitive/move_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(MovePrimTest, primitive_name_test)
{
    MovePrimitive move_prim = MovePrimitive(0, Point(), Angle(), 0.0);

    EXPECT_EQ("Move Primitive", move_prim.getPrimitiveName());
}

TEST(MovePrimTest, get_robot_id_test)
{
    unsigned int robot_id = 4U;

    MovePrimitive move_prim = MovePrimitive(robot_id, Point(), Angle(), 0.0);

    EXPECT_EQ(robot_id, move_prim.getRobotId());
}

TEST(MovePrimTest, parameter_array_test)
{
    const Point destination     = Point(-1, 2);
    const Angle final_angle     = Angle::ofRadians(3.15);
    const double final_speed    = 2.11;
    const unsigned int robot_id = 2U;

    MovePrimitive move_prim =
        MovePrimitive(robot_id, destination, final_angle, final_speed);

    std::vector<double> param_array = move_prim.getParameters();

    EXPECT_DOUBLE_EQ(destination.x(), param_array[0]);
    EXPECT_DOUBLE_EQ(destination.y(), param_array[1]);
    EXPECT_DOUBLE_EQ(final_angle.toRadians(), param_array[2]);
    EXPECT_DOUBLE_EQ(final_speed, param_array[3]);
}

TEST(MovePrimTest, get_final_speed_test)
{
    const double final_speed = 2.11;

    MovePrimitive move_prim = MovePrimitive(0, Point(), Angle(), final_speed);

    EXPECT_DOUBLE_EQ(move_prim.getFinalSpeed(), final_speed);
}

TEST(MovePrimTest, get_final_orientation_test)
{
    const Angle final_angle = Angle::ofRadians(3.15);

    MovePrimitive move_prim = MovePrimitive(0, Point(), final_angle, 0.0);

    EXPECT_EQ(move_prim.getFinalAngle(), final_angle);
}

TEST(MovePrimTest, get_final_destination_test)
{
    const Point destination = Point(-1, 2);

    MovePrimitive move_prim = MovePrimitive(0, destination, Angle(), 0.0);

    EXPECT_EQ(move_prim.getDestination(), destination);
}

TEST(MovePrimTest, get_extra_bit_array_test)
{
    MovePrimitive move_prim = MovePrimitive(0, Point(), Angle(), 0.0);

    std::vector<bool> extra_bit_array = move_prim.getExtraBits();

    EXPECT_EQ(extra_bit_array, std::vector<bool>());
}

TEST(MovePrimitiveTest, create_primitive_from_message_test)
{
    const Point destination     = Point(2, -3);
    const Angle final_angle     = Angle::ofRadians(3.55);
    const double final_speed    = 2.22;
    const unsigned int robot_id = 3U;

    MovePrimitive move_prim =
        MovePrimitive(robot_id, destination, final_angle, final_speed);

    thunderbots_msgs::Primitive prim_message = move_prim.createMsg();

    MovePrimitive new_prim = MovePrimitive(prim_message);

    std::vector<double> parameters = new_prim.getParameters();

    EXPECT_EQ(MovePrimitive::PRIMITIVE_NAME, new_prim.getPrimitiveName());
    EXPECT_EQ(new_prim, move_prim);
}

TEST(MovePrimitiveTest, test_equality_operator_primitives_equal)
{
    MovePrimitive move_prim       = MovePrimitive(0, Point(), Angle(), 0.0);
    MovePrimitive move_prim_other = MovePrimitive(0, Point(), Angle(), 0.0);

    EXPECT_EQ(move_prim, move_prim_other);
}

TEST(MovePrimitiveTest, test_inequality_operator_with_mismatched_robot_id)
{
    MovePrimitive move_prim       = MovePrimitive(0, Point(), Angle(), 0.0);
    MovePrimitive move_prim_other = MovePrimitive(4, Point(), Angle(), 0.0);

    EXPECT_NE(move_prim, move_prim_other);
}

TEST(MovePrimitiveTest, test_inequality_operator_with_mismatched_dest)
{
    MovePrimitive move_prim       = MovePrimitive(0, Point(), Angle(), 0.0);
    MovePrimitive move_prim_other = MovePrimitive(0, Point(4, 4), Angle(), 0.0);

    EXPECT_NE(move_prim, move_prim_other);
}

TEST(MovePrimitiveTest, test_inequality_operator_with_mismatched_final_angle)
{
    MovePrimitive move_prim       = MovePrimitive(0, Point(), Angle::quarter(), 0.0);
    MovePrimitive move_prim_other = MovePrimitive(0, Point(), Angle(), 0.0);

    EXPECT_NE(move_prim, move_prim_other);
}

TEST(MovePrimitiveTest, test_inequality_operator_with_mismatched_final_speed)
{
    MovePrimitive move_prim       = MovePrimitive(0, Point(), Angle(), 0.0);
    MovePrimitive move_prim_other = MovePrimitive(0, Point(), Angle(), 3.2);

    EXPECT_NE(move_prim, move_prim_other);
}

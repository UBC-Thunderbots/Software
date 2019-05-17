/**
 * This file contains the unit tests for the DribblePrimitive class
 */

#include "ai/primitive/dribble_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(DribblePrimTest, primitive_name_test)
{
    DribblePrimitive dribble_prim = DribblePrimitive(0, Point(), Angle(), 0.0, false);

    EXPECT_EQ("Dribble Primitive", dribble_prim.getPrimitiveName());
}

TEST(DribblePrimTest, get_robot_id_test)
{
    unsigned int robot_id = 4U;

    DribblePrimitive dribble_prim =
        DribblePrimitive(robot_id, Point(), Angle(), 0.0, false);

    EXPECT_EQ(robot_id, dribble_prim.getRobotId());
}

TEST(DribblePrimTest, parameter_array_test)
{
    const Point destination     = Point(-1, 2);
    const Angle final_angle     = Angle::ofRadians(3.15);
    const unsigned int robot_id = 2U;
    const double rpm            = 3.14;

    DribblePrimitive dribble_prim =
        DribblePrimitive(robot_id, destination, final_angle, rpm, false);

    std::vector<double> param_array = dribble_prim.getParameters();

    EXPECT_DOUBLE_EQ(destination.x(), param_array[0]);
    EXPECT_DOUBLE_EQ(destination.y(), param_array[1]);
    EXPECT_DOUBLE_EQ(final_angle.toRadians(), param_array[2]);
    EXPECT_DOUBLE_EQ(rpm, param_array[3]);
}

TEST(DribblePrimTest, get_final_orientation_test)
{
    const Angle final_angle = Angle::ofRadians(3.15);

    DribblePrimitive dribble_prim = DribblePrimitive(0, Point(), final_angle, 0.0, false);

    EXPECT_EQ(dribble_prim.getFinalAngle(), final_angle);
}

TEST(DribblePrimTest, get_final_destination_test)
{
    const Point destination = Point(-1, 2);

    DribblePrimitive dribble_prim = DribblePrimitive(0, destination, Angle(), 0.0, false);

    EXPECT_EQ(dribble_prim.getDestination(), destination);
}

TEST(DribblePrimTest, get_rpm_test)
{
    const double rpm = 40.5;

    DribblePrimitive dribble_prim = DribblePrimitive(0, Point(), Angle(), rpm, false);

    EXPECT_EQ(dribble_prim.getRpm(), rpm);
}

TEST(DribblePrimTest, get_extra_bit_array_test)
{
    const bool small_kick_allowed = true;

    DribblePrimitive dribble_prim =
        DribblePrimitive(0, Point(), Angle(), 0.0, small_kick_allowed);

    std::vector<bool> extra_bit_array = dribble_prim.getExtraBits();

    EXPECT_TRUE(extra_bit_array[0]);
}

TEST(DribblePrimTest, is_small_kick_allowed_test)
{
    const bool small_kick_allowed = false;

    DribblePrimitive dribble_prim =
        DribblePrimitive(0, Point(), Angle(), 0.0, small_kick_allowed);

    EXPECT_FALSE(dribble_prim.isSmallKickAllowed());
}

TEST(DribblePrimTest, create_primitive_from_message_test)
{
    const Point destination       = Point(2, -3);
    const Angle final_angle       = Angle::ofRadians(3.55);
    const unsigned int robot_id   = 3U;
    const double rpm              = 30.5;
    const bool small_kick_allowed = true;

    DribblePrimitive dribble_prim =
        DribblePrimitive(robot_id, destination, final_angle, rpm, small_kick_allowed);

    thunderbots_msgs::Primitive prim_message = dribble_prim.createMsg();

    DribblePrimitive new_prim = DribblePrimitive(prim_message);

    std::vector<double> parameters = new_prim.getParameters();

    std::vector<bool> extraBits = new_prim.getExtraBits();

    EXPECT_EQ(DribblePrimitive::PRIMITIVE_NAME, new_prim.getPrimitiveName());
    EXPECT_EQ(new_prim, dribble_prim);
}

TEST(DribblePrimTest, test_equality_operator_primitives_equal)
{
    DribblePrimitive dribble_prim = DribblePrimitive(0, Point(), Angle(), 0.0, false);
    DribblePrimitive dribble_prim_other =
        DribblePrimitive(0, Point(), Angle(), 0.0, false);

    EXPECT_EQ(dribble_prim, dribble_prim_other);
}

TEST(DribblePrimTest, test_inequality_operator_with_mismatched_robot_id)
{
    DribblePrimitive dribble_prim = DribblePrimitive(0, Point(), Angle(), 0.0, false);
    DribblePrimitive dribble_prim_other =
        DribblePrimitive(7, Point(), Angle(), 0.0, false);

    EXPECT_NE(dribble_prim, dribble_prim_other);
}

TEST(DribblePrimTest, test_inequality_operator_with_mismatched_dest)
{
    DribblePrimitive dribble_prim = DribblePrimitive(0, Point(), Angle(), 0.0, false);
    DribblePrimitive dribble_prim_other =
        DribblePrimitive(0, Point(-4.0, 0), Angle(), 0.0, false);

    EXPECT_NE(dribble_prim, dribble_prim_other);
}

TEST(DribblePrimTest, test_inequality_operator_with_mismatched_final_angle)
{
    DribblePrimitive dribble_prim =
        DribblePrimitive(0, Point(), Angle::threeQuarter(), 0.0, false);
    DribblePrimitive dribble_prim_other =
        DribblePrimitive(0, Point(), Angle(), 0.0, false);

    EXPECT_NE(dribble_prim, dribble_prim_other);
}

TEST(DribblePrimTest, test_inequality_operator_with_mismatched_rpm)
{
    DribblePrimitive dribble_prim = DribblePrimitive(0, Point(), Angle(), 500, false);
    DribblePrimitive dribble_prim_other =
        DribblePrimitive(0, Point(), Angle(), 698, false);

    EXPECT_NE(dribble_prim, dribble_prim_other);
}

TEST(DribblePrimTest, test_inequality_operator_with_mismatched_small_kick_allowed)
{
    DribblePrimitive dribble_prim = DribblePrimitive(0, Point(), Angle(), 0.0, false);
    DribblePrimitive dribble_prim_other =
        DribblePrimitive(0, Point(), Angle(), 0.0, true);

    EXPECT_NE(dribble_prim, dribble_prim_other);
}

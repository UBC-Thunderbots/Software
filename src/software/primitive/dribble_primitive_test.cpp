/**
 * This file contains the unit tests for the DribblePrimitive class
 */

#include "software/primitive/dribble_primitive.h"

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

TEST(DribblePrimTest, get_final_orientation_test)
{
    const Angle final_angle = Angle::fromRadians(3.15);

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

TEST(DribblePrimTest, is_small_kick_allowed_test)
{
    const bool small_kick_allowed = false;

    DribblePrimitive dribble_prim =
        DribblePrimitive(0, Point(), Angle(), 0.0, small_kick_allowed);

    EXPECT_FALSE(dribble_prim.isSmallKickAllowed());
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

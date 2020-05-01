/**
 * This file contains the unit tests for the PivotPrimitive class
 */

#include "software/primitive/pivot_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(PivotPrimTest, primitive_name_test)
{
    PivotPrimitive pivot_prim = PivotPrimitive(0, Point(), Angle(), Angle(), false);

    EXPECT_EQ("Pivot Primitive", pivot_prim.getPrimitiveName());
}

TEST(PivotPrimTest, get_robot_id_test)
{
    unsigned int robot_id = 4U;

    PivotPrimitive pivot_prim = PivotPrimitive(4, Point(), Angle(), Angle(), false);

    EXPECT_EQ(robot_id, pivot_prim.getRobotId());
}

TEST(PivotPrimTest, get_pivot_point_test)
{
    Point pivot_point = Point(-4, 3);

    PivotPrimitive pivot_prim = PivotPrimitive(0, pivot_point, Angle(), Angle(), false);

    EXPECT_EQ(pivot_point, pivot_prim.getPivotPoint());
}

TEST(PivotPrimTest, get_final_angle_test)
{
    Angle final_angle = Angle::fromRadians(1.51);

    PivotPrimitive pivot_prim = PivotPrimitive(0, Point(), final_angle, Angle(), false);

    EXPECT_EQ(final_angle, pivot_prim.getFinalAngle());
}

TEST(PivotPrimTest, get_pivot_speed_test)
{
    Angle pivot_speed = Angle::fromRadians(1.2);

    PivotPrimitive pivot_prim = PivotPrimitive(0, Point(), Angle(), pivot_speed, true);

    EXPECT_DOUBLE_EQ(pivot_speed.toRadians(), pivot_prim.getPivotSpeed().toRadians());
}

TEST(PivotPrimTest, test_equality_operator_primitives_equal)
{
    PivotPrimitive pivot_prim       = PivotPrimitive(0, Point(), Angle(), Angle(), true);
    PivotPrimitive pivot_prim_other = PivotPrimitive(0, Point(), Angle(), Angle(), true);

    EXPECT_EQ(pivot_prim, pivot_prim_other);
}

TEST(PivotPrimTest, test_inequality_operator_with_mismatched_robot_id)
{
    PivotPrimitive pivot_prim       = PivotPrimitive(0, Point(), Angle(), Angle(), true);
    PivotPrimitive pivot_prim_other = PivotPrimitive(8, Point(), Angle(), Angle(), true);

    EXPECT_NE(pivot_prim, pivot_prim_other);
}

TEST(PivotPrimTest, test_inequality_operator_with_mismatched_pivot_point)
{
    PivotPrimitive pivot_prim = PivotPrimitive(0, Point(), Angle(), Angle(), true);
    PivotPrimitive pivot_prim_other =
        PivotPrimitive(0, Point(1, 1), Angle(), Angle(), true);

    EXPECT_NE(pivot_prim, pivot_prim_other);
}

TEST(PivotPrimTest, test_inequality_operator_with_mismatched_final_angle)
{
    PivotPrimitive pivot_prim = PivotPrimitive(0, Point(), Angle(), Angle(), true);
    PivotPrimitive pivot_prim_other =
        PivotPrimitive(0, Point(), Angle::threeQuarter(), Angle(), true);

    EXPECT_NE(pivot_prim, pivot_prim_other);
}

TEST(PivotPrimTest, test_inequality_operator_with_mismatched_pivot_speed)
{
    PivotPrimitive pivot_prim = PivotPrimitive(0, Point(), Angle(), Angle(), true);
    PivotPrimitive pivot_prim_other =
        PivotPrimitive(0, Point(), Angle(), Angle::threeQuarter(), true);

    EXPECT_NE(pivot_prim, pivot_prim_other);
}

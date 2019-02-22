/**
 * This file contains the unit tests for the PivotPrimitive class
 *
 */

#include "ai/primitive/pivot_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(PivotPrimTest, primitive_name_test)
{
    PivotPrimitive pivot_prim = PivotPrimitive(0, Point(), Angle(), 0);

    EXPECT_EQ("Pivot Primitive", pivot_prim.getPrimitiveName());
}

TEST(PivotPrimTest, get_robot_id_test)
{
    unsigned int robot_id = 4U;

    PivotPrimitive pivot_prim = PivotPrimitive(robot_id, Point(), Angle(), 0);

    EXPECT_EQ(robot_id, pivot_prim.getRobotId());
}

TEST(PivotPrimTest, parameter_array_test)
{
    const unsigned int robot_id = 6U;
    const Point pivot_point     = Point(-1, 2);
    const Angle final_angle     = Angle::ofRadians(1.5);
    const double pivot_radius   = 2.5;

    PivotPrimitive pivot_prim =
        PivotPrimitive(robot_id, pivot_point, final_angle, pivot_radius);

    std::vector<double> param_array = pivot_prim.getParameters();

    EXPECT_DOUBLE_EQ(pivot_point.x(), param_array[0]);
    EXPECT_DOUBLE_EQ(pivot_point.y(), param_array[1]);
    EXPECT_DOUBLE_EQ(final_angle.toRadians(), param_array[2]);
    EXPECT_DOUBLE_EQ(pivot_radius, param_array[3]);
}

TEST(PivotPrimTest, get_pivot_point_test)
{
    Point pivot_point = Point(-4, 3);

    PivotPrimitive pivot_prim = PivotPrimitive(0, pivot_point, Angle(), 0);

    EXPECT_EQ(pivot_point, pivot_prim.getPivotPoint());
}

TEST(PivotPrimTest, get_final_angle_test)
{
    Angle final_angle = Angle::ofRadians(1.51);

    PivotPrimitive pivot_prim = PivotPrimitive(0, Point(), final_angle, 0);

    EXPECT_EQ(final_angle, pivot_prim.getFinalAngle());
}

TEST(PivotPrimTest, get_pivot_radius_test)
{
    double pivot_radius = 1.85;

    PivotPrimitive pivot_prim = PivotPrimitive(0, Point(), Angle(), pivot_radius);

    EXPECT_DOUBLE_EQ(pivot_radius, pivot_prim.getPivotRadius());
}

TEST(PivotPrimTest, get_extra_bit_array_test)
{
    PivotPrimitive pivot_prim = PivotPrimitive(0, Point(), Angle(), 0);

    std::vector<bool> extra_bit_array = pivot_prim.getExtraBits();

    EXPECT_EQ(extra_bit_array, std::vector<bool>());
}

TEST(PivotPrimTest, test_equality_operator_primitives_equal)
{
    PivotPrimitive pivot_prim       = PivotPrimitive(0, Point(), Angle(), 0);
    PivotPrimitive pivot_prim_other = PivotPrimitive(0, Point(), Angle(), 0);

    EXPECT_EQ(pivot_prim, pivot_prim_other);
}

TEST(PivotPrimTest, test_inequality_operator_with_mismatched_robot_id)
{
    PivotPrimitive pivot_prim       = PivotPrimitive(0, Point(), Angle(), 0);
    PivotPrimitive pivot_prim_other = PivotPrimitive(8, Point(), Angle(), 0);

    EXPECT_NE(pivot_prim, pivot_prim_other);
}

TEST(PivotPrimTest, test_inequality_operator_with_mismatched_pivot_point)
{
    PivotPrimitive pivot_prim       = PivotPrimitive(0, Point(), Angle(), 0);
    PivotPrimitive pivot_prim_other = PivotPrimitive(0, Point(1, 1), Angle(), 0);

    EXPECT_NE(pivot_prim, pivot_prim_other);
}

TEST(PivotPrimTest, test_inequality_operator_with_mismatched_final_angle)
{
    PivotPrimitive pivot_prim = PivotPrimitive(0, Point(), Angle(), 0);
    PivotPrimitive pivot_prim_other =
        PivotPrimitive(0, Point(), Angle::threeQuarter(), 0);

    EXPECT_NE(pivot_prim, pivot_prim_other);
}

TEST(PivotPrimTest, test_inequality_operator_with_mismatched_pivot_radius)
{
    PivotPrimitive pivot_prim       = PivotPrimitive(0, Point(), Angle(), 1);
    PivotPrimitive pivot_prim_other = PivotPrimitive(0, Point(), Angle(), 1.25);

    EXPECT_NE(pivot_prim, pivot_prim_other);
}

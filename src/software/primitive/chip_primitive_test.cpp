/**
 * This file contains the unit tests for the ChipPrimitive class
 */

#include "software/primitive/chip_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(ChipPrimTest, primitive_name_test)
{
    ChipPrimitive chip_prim = ChipPrimitive(0, Point(), Angle(), 0.0);

    EXPECT_EQ("Chip Primitive", chip_prim.getPrimitiveName());
}

TEST(ChipPrimTest, get_robot_id_test)
{
    unsigned int robot_id = 2U;

    ChipPrimitive chip_prim = ChipPrimitive(robot_id, Point(), Angle(), 0.0);

    EXPECT_EQ(robot_id, chip_prim.getRobotId());
}

TEST(ChipPrimTest, get_chip_origin_test)
{
    const Point chip_origin = Point(3, 5);

    ChipPrimitive chip_prim = ChipPrimitive(0, chip_origin, Angle(), 0.0);

    EXPECT_EQ(chip_prim.getChipOrigin(), chip_origin);
}

TEST(ChipPrimTest, get_chip_direction_test)
{
    const Angle chip_direction = Angle::fromRadians(1.35);

    ChipPrimitive chip_prim = ChipPrimitive(0, Point(), chip_direction, 0.0);

    EXPECT_EQ(chip_prim.getChipDirection(), chip_direction);
}

TEST(ChipPrimTest, get_chip_distance_test)
{
    const double chip_distance_meters = 1.23;

    ChipPrimitive chip_prim = ChipPrimitive(0, Point(), Angle(), chip_distance_meters);

    EXPECT_DOUBLE_EQ(chip_prim.getChipDistance(), chip_distance_meters);
}

TEST(ChipPrimitiveTest, test_equality_operator_primitives_equal)
{
    ChipPrimitive chip_prim       = ChipPrimitive(0, Point(), Angle(), 0.0);
    ChipPrimitive chip_prim_other = ChipPrimitive(0, Point(), Angle(), 0.0);

    EXPECT_EQ(chip_prim, chip_prim_other);
}

TEST(ChipPrimitiveTest, test_inequality_operator_with_mismatched_robot_ids)
{
    ChipPrimitive chip_prim       = ChipPrimitive(1, Point(), Angle(), 0.0);
    ChipPrimitive chip_prim_other = ChipPrimitive(4, Point(), Angle(), 0.0);

    EXPECT_NE(chip_prim, chip_prim_other);
}

TEST(ChipPrimitiveTest, test_inequality_operator_with_mismatched_chip_origins)
{
    ChipPrimitive chip_prim       = ChipPrimitive(0, Point(), Angle(), 0.0);
    ChipPrimitive chip_prim_other = ChipPrimitive(0, Point(1, -5), Angle(), 0.0);

    EXPECT_NE(chip_prim, chip_prim_other);
}

TEST(ChipPrimitiveTest, test_inequality_operator_with_mismatched_chip_directions)
{
    ChipPrimitive chip_prim       = ChipPrimitive(0, Point(), Angle::quarter(), 0.0);
    ChipPrimitive chip_prim_other = ChipPrimitive(0, Point(), Angle::half(), 0.0);

    EXPECT_NE(chip_prim, chip_prim_other);
}

TEST(ChipPrimitiveTest, test_inequality_operator_with_mismatched_chip_distance)
{
    ChipPrimitive chip_prim       = ChipPrimitive(0, Point(), Angle(), 1.0);
    ChipPrimitive chip_prim_other = ChipPrimitive(0, Point(), Angle(), 3.14);

    EXPECT_NE(chip_prim, chip_prim_other);
}

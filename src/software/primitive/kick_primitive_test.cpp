/**
 * This file contains the unit tests for the KickPrimitive class
 */

#include "software/primitive/kick_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(KickPrimTest, primitive_name_test)
{
    KickPrimitive kick_prim = KickPrimitive(0, Point(), Angle(), 0.0);

    EXPECT_EQ("Kick Primitive", kick_prim.getPrimitiveName());
}

TEST(KickPrimTest, get_robot_id_test)
{
    unsigned int robot_id = 1U;

    KickPrimitive kick_prim = KickPrimitive(robot_id, Point(), Angle(), 0.0);

    EXPECT_EQ(robot_id, kick_prim.getRobotId());
}

TEST(KickPrimTest, get_kick_origin_test)
{
    const Point kick_origin = Point(5, 0);

    KickPrimitive kick_prim = KickPrimitive(0, kick_origin, Angle(), 0.0);

    EXPECT_EQ(kick_prim.getKickOrigin(), kick_origin);
}

TEST(KickPrimTest, get_kick_direction_test)
{
    const Angle kick_direction = Angle::fromRadians(2.58);

    KickPrimitive kick_prim = KickPrimitive(0, Point(), kick_direction, 0.0);

    EXPECT_EQ(kick_prim.getKickDirection(), kick_direction);
}

TEST(KickPrimTest, get_kick_speed_test)
{
    const double kick_speed_meters_per_second = 3.33;

    KickPrimitive kick_prim =
        KickPrimitive(0, Point(), Angle(), kick_speed_meters_per_second);

    EXPECT_DOUBLE_EQ(kick_prim.getKickSpeed(), kick_speed_meters_per_second);
}

TEST(KickPrimitiveTest, test_equality_operator_primitives_equal)
{
    KickPrimitive kick_prim       = KickPrimitive(0, Point(), Angle(), 0.0);
    KickPrimitive kick_prim_other = KickPrimitive(0, Point(), Angle(), 0.0);

    EXPECT_EQ(kick_prim, kick_prim_other);
}

TEST(KickPrimitiveTest, test_inequality_operator_with_mismatched_robot_id)
{
    KickPrimitive kick_prim       = KickPrimitive(0, Point(), Angle(), 0.0);
    KickPrimitive kick_prim_other = KickPrimitive(10, Point(), Angle(), 0.0);

    EXPECT_NE(kick_prim, kick_prim_other);
}

TEST(KickPrimitiveTest, test_inequality_operator_with_mismatched_kick_origin)
{
    KickPrimitive kick_prim       = KickPrimitive(0, Point(), Angle(), 0.0);
    KickPrimitive kick_prim_other = KickPrimitive(0, Point(0.1, 0.34), Angle(), 0.0);

    EXPECT_NE(kick_prim, kick_prim_other);
}

TEST(KickPrimitiveTest, test_inequality_operator_with_mismatched_kick_direction)
{
    KickPrimitive kick_prim       = KickPrimitive(0, Point(), Angle::quarter(), 0.0);
    KickPrimitive kick_prim_other = KickPrimitive(0, Point(), Angle::full(), 0.0);

    EXPECT_NE(kick_prim, kick_prim_other);
}

TEST(KickPrimitiveTest, test_inequality_operator_with_mismatched_kick_speed)
{
    KickPrimitive kick_prim       = KickPrimitive(0, Point(), Angle(), 0.0);
    KickPrimitive kick_prim_other = KickPrimitive(0, Point(), Angle(), 3.08);

    EXPECT_NE(kick_prim, kick_prim_other);
}

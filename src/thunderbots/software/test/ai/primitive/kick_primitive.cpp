/**
 * This file contains the unit tests for the KickPrimitive class
 */

#include "ai/primitive/kick_primitive.h"

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

TEST(KickPrimTest, parameter_array_test)
{
    const unsigned int robot_id               = 1U;
    const Point kick_origin                   = Point(5, 0);
    const Angle kick_direction                = Angle::ofRadians(2.58);
    const double kick_speed_meters_per_second = 3.33;

    KickPrimitive kick_prim = KickPrimitive(robot_id, kick_origin, kick_direction,
                                            kick_speed_meters_per_second);

    std::vector<double> param_array = kick_prim.getParameters();

    EXPECT_DOUBLE_EQ(kick_origin.x(), param_array[0]);
    EXPECT_DOUBLE_EQ(kick_origin.y(), param_array[1]);
    EXPECT_DOUBLE_EQ(kick_direction.toRadians(), param_array[2]);
    EXPECT_DOUBLE_EQ(kick_speed_meters_per_second, param_array[3]);
}

TEST(KickPrimTest, get_kick_origin_test)
{
    const Point kick_origin = Point(5, 0);

    KickPrimitive kick_prim = KickPrimitive(0, kick_origin, Angle(), 0.0);

    EXPECT_EQ(kick_prim.getKickOrigin(), kick_origin);
}

TEST(KickPrimTest, get_kick_direction_test)
{
    const Angle kick_direction = Angle::ofRadians(2.58);

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

TEST(KickPrimTest, get_extra_bit_array_test)
{
    KickPrimitive kick_prim = KickPrimitive(0, Point(), Angle(), 0.0);

    std::vector<bool> extra_bit_array = kick_prim.getExtraBits();

    EXPECT_EQ(extra_bit_array, std::vector<bool>());
}

TEST(KickPrimitiveTest, create_primitive_from_message_test)
{
    const unsigned int robot_id               = 1U;
    const Point kick_origin                   = Point(5, 0);
    const Angle kick_direction                = Angle::ofRadians(2.58);
    const double kick_speed_meters_per_second = 3.33;

    KickPrimitive kick_prim = KickPrimitive(robot_id, kick_origin, kick_direction,
                                            kick_speed_meters_per_second);

    thunderbots_msgs::Primitive prim_message = kick_prim.createMsg();

    KickPrimitive new_prim = KickPrimitive(prim_message);

    std::vector<double> parameters = new_prim.getParameters();

    EXPECT_EQ(KickPrimitive::PRIMITIVE_NAME, new_prim.getPrimitiveName());
    EXPECT_EQ(new_prim, kick_prim);
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

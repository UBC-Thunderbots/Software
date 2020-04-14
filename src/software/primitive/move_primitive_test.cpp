/**
 * This file contains the unit tests for the MovePrimitive class
 */

#include "software/primitive/move_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(MovePrimTest, primitive_name_test)
{
    MovePrimitive move_prim = MovePrimitive(0, Point(), Angle(), 0.0, DribblerEnable::OFF,
                                            MoveType::NORMAL, AutokickType::NONE);

    EXPECT_EQ("Move Primitive", move_prim.getPrimitiveName());
}

TEST(MovePrimTest, get_robot_id_test)
{
    unsigned int robot_id = 4U;

    MovePrimitive move_prim =
        MovePrimitive(robot_id, Point(), Angle(), 0.0, DribblerEnable::OFF,
                      MoveType::NORMAL, AutokickType::NONE);

    EXPECT_EQ(robot_id, move_prim.getRobotId());
}

TEST(MovePrimTest, autokick_and_dribble_disabled_by_default)
{
    MovePrimitive move_prim = MovePrimitive(0, Point(), Angle(), 0.0, DribblerEnable::OFF,
                                            MoveType::NORMAL, AutokickType::NONE);
    EXPECT_FALSE(move_prim.getAutoKickType());
    EXPECT_FALSE(move_prim.getDribblerEnable() == DribblerEnable::ON);
}

TEST(MovePrimTest, get_dribble_enabled)
{
    MovePrimitive move_prim = MovePrimitive(0, Point(), Angle(), 0.0, DribblerEnable::ON,
                                            MoveType::NORMAL, AutokickType::NONE);
    EXPECT_TRUE(move_prim.getDribblerEnable() == DribblerEnable::ON);
    EXPECT_EQ(move_prim.getAutoKickType(), NONE);
}

TEST(MovePrimTest, get_slow_enabled)
{
    MovePrimitive move_prim = MovePrimitive(0, Point(), Angle(), 0.0, DribblerEnable::ON,
                                            MoveType::SLOW, AutokickType::NONE);
    EXPECT_FALSE(move_prim.getMoveType() == MoveType::NORMAL);
}

TEST(MovePrimTest, get_autokick_enabled)
{
    MovePrimitive move_prim = MovePrimitive(0, Point(), Angle(), 0.0, DribblerEnable::OFF,
                                            MoveType::NORMAL, AutokickType::AUTOKICK);
    EXPECT_FALSE(move_prim.getDribblerEnable() == DribblerEnable::ON);
    EXPECT_EQ(move_prim.getAutoKickType(), AUTOKICK);
}

TEST(MovePrimTest, get_autochip_enabled)
{
    MovePrimitive move_prim = MovePrimitive(0, Point(), Angle(), 0.0, DribblerEnable::OFF,
                                            MoveType::NORMAL, AutokickType::AUTOCHIP);
    EXPECT_FALSE(move_prim.getDribblerEnable() == DribblerEnable::ON);
    EXPECT_EQ(move_prim.getAutoKickType(), AUTOCHIP);
}

TEST(MovePrimTest, get_final_speed_test)
{
    const double final_speed = 2.11;

    MovePrimitive move_prim =
        MovePrimitive(0, Point(), Angle(), final_speed, DribblerEnable::OFF,
                      MoveType::NORMAL, AutokickType::NONE);

    EXPECT_DOUBLE_EQ(move_prim.getFinalSpeed(), final_speed);
}

TEST(MovePrimTest, get_final_orientation_test)
{
    const Angle final_angle = Angle::fromRadians(3.15);

    MovePrimitive move_prim =
        MovePrimitive(0, Point(), final_angle, 0.0, DribblerEnable::OFF, MoveType::NORMAL,
                      AutokickType::NONE);

    EXPECT_EQ(move_prim.getFinalAngle(), final_angle);
}

TEST(MovePrimTest, get_final_destination_test)
{
    const Point destination = Point(-1, 2);

    MovePrimitive move_prim =
        MovePrimitive(0, destination, Angle(), 0.0, DribblerEnable::OFF, MoveType::NORMAL,
                      AutokickType::NONE);

    EXPECT_EQ(move_prim.getDestination(), destination);
}

TEST(MovePrimitiveTest, test_equality_operator_primitives_equal)
{
    MovePrimitive move_prim = MovePrimitive(0, Point(), Angle(), 0.0, DribblerEnable::OFF,
                                            MoveType::NORMAL, AutokickType::NONE);
    MovePrimitive move_prim_other =
        MovePrimitive(0, Point(), Angle(), 0.0, DribblerEnable::OFF, MoveType::NORMAL,
                      AutokickType::NONE);

    EXPECT_EQ(move_prim, move_prim_other);
}

TEST(MovePrimitiveTest, test_inequality_operator_with_mismatched_robot_id)
{
    MovePrimitive move_prim = MovePrimitive(0, Point(), Angle(), 0.0, DribblerEnable::OFF,
                                            MoveType::NORMAL, AutokickType::NONE);
    MovePrimitive move_prim_other =
        MovePrimitive(4, Point(), Angle(), 0.0, DribblerEnable::OFF, MoveType::NORMAL,
                      AutokickType::NONE);

    EXPECT_NE(move_prim, move_prim_other);
}

TEST(MovePrimitiveTest, test_inequality_operator_with_mismatched_dest)
{
    MovePrimitive move_prim = MovePrimitive(0, Point(), Angle(), 0.0, DribblerEnable::OFF,
                                            MoveType::NORMAL, AutokickType::NONE);
    MovePrimitive move_prim_other =
        MovePrimitive(0, Point(4, 4), Angle(), 0.0, DribblerEnable::OFF, MoveType::NORMAL,
                      AutokickType::NONE);

    EXPECT_NE(move_prim, move_prim_other);
}

TEST(MovePrimitiveTest, test_inequality_operator_with_mismatched_final_angle)
{
    MovePrimitive move_prim =
        MovePrimitive(0, Point(), Angle::quarter(), 0.0, DribblerEnable::OFF,
                      MoveType::NORMAL, AutokickType::NONE);
    MovePrimitive move_prim_other =
        MovePrimitive(0, Point(), Angle(), 0.0, DribblerEnable::OFF, MoveType::NORMAL,
                      AutokickType::NONE);

    EXPECT_NE(move_prim, move_prim_other);
}

TEST(MovePrimitiveTest, test_inequality_operator_with_mismatched_final_speed)
{
    MovePrimitive move_prim = MovePrimitive(0, Point(), Angle(), 0.0, DribblerEnable::OFF,
                                            MoveType::NORMAL, AutokickType::NONE);
    MovePrimitive move_prim_other =
        MovePrimitive(0, Point(), Angle(), 3.2, DribblerEnable::OFF, MoveType::NORMAL,
                      AutokickType::NONE);

    EXPECT_NE(move_prim, move_prim_other);
}

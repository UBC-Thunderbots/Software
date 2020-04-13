/**
 * This file contains the unit tests for the MoveSpinPrimitive class
 */

#include "software/primitive/movespin_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(MoveSpinPrimTest, primitive_name_test)
{
    MoveSpinPrimitive movespin_prim =
        MoveSpinPrimitive(0, Point(), AngularVelocity(), 1.0);

    EXPECT_EQ("MoveSpin Primitive", movespin_prim.getPrimitiveName());
}

TEST(MoveSpinPrimTest, get_robot_id_test)
{
    unsigned int robot_id = 3U;

    MoveSpinPrimitive movespin_prim =
        MoveSpinPrimitive(robot_id, Point(), AngularVelocity(), 1.0);

    EXPECT_EQ(robot_id, movespin_prim.getRobotId());
}

TEST(MoveSpinPrimTest, get_angular_vel_test)
{
    const AngularVelocity angular_vel = AngularVelocity::fromRadians(2.5);

    MoveSpinPrimitive movespin_prim = MoveSpinPrimitive(0, Point(), angular_vel, 1.0);

    EXPECT_EQ(movespin_prim.getAngularVelocity(), angular_vel);
}

TEST(MoveSpinPrimTest, get_final_destination_test)
{
    const Point destination = Point(1, -2);

    MoveSpinPrimitive movespin_prim =
        MoveSpinPrimitive(0, destination, AngularVelocity(), 1.0);

    EXPECT_EQ(movespin_prim.getDestination(), destination);
}

TEST(MoveSpinPrimTest, get_final_speed_test)
{
    const double final_speed = 2.0;

    MoveSpinPrimitive movespin_prim =
        MoveSpinPrimitive(0, Point(), AngularVelocity(), final_speed);

    EXPECT_DOUBLE_EQ(movespin_prim.getFinalSpeed(), final_speed);
}

TEST(MoveSpinPrimTest, test_equality_operator_primitives_equal)
{
    MoveSpinPrimitive movespin_prim =
        MoveSpinPrimitive(0, Point(), AngularVelocity(), 1.0);
    MoveSpinPrimitive movespin_prim_other =
        MoveSpinPrimitive(0, Point(), AngularVelocity(), 1.0);

    EXPECT_EQ(movespin_prim, movespin_prim_other);
}

TEST(MoveSpinPrimTest, test_inequality_operator_with_mismatched_robot_id)
{
    MoveSpinPrimitive movespin_prim =
        MoveSpinPrimitive(0, Point(), AngularVelocity(), 1.0);
    MoveSpinPrimitive movespin_prim_other =
        MoveSpinPrimitive(6, Point(), AngularVelocity(), 1.0);

    EXPECT_NE(movespin_prim, movespin_prim_other);
}

TEST(MoveSpinPrimTest, test_inequality_operator_with_mismatched_dest)
{
    MoveSpinPrimitive movespin_prim =
        MoveSpinPrimitive(0, Point(), AngularVelocity(), 1.0);
    MoveSpinPrimitive movespin_prim_other =
        MoveSpinPrimitive(0, Point(-0.3, 5), AngularVelocity(), 1.0);

    EXPECT_NE(movespin_prim, movespin_prim_other);
}

TEST(MoveSpinPrimTest, test_inequality_operator_with_mismatched_angular_velocity)
{
    MoveSpinPrimitive movespin_prim =
        MoveSpinPrimitive(0, Point(), AngularVelocity(), 1.0);
    MoveSpinPrimitive movespin_prim_other =
        MoveSpinPrimitive(0, Point(), AngularVelocity::half(), 1.0);

    EXPECT_NE(movespin_prim, movespin_prim_other);
}

TEST(MoveSpinPrimTest, test_inequality_operator_with_mismatched_final_speed)
{
    MoveSpinPrimitive movespin_prim =
        MoveSpinPrimitive(0, Point(), AngularVelocity(), 1.0);
    MoveSpinPrimitive movespin_prim_other =
        MoveSpinPrimitive(0, Point(), AngularVelocity(), 0.5);

    EXPECT_NE(movespin_prim, movespin_prim_other);
}

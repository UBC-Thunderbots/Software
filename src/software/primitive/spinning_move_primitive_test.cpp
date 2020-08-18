#include "software/primitive/spinning_move_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

TEST(SpinningMovePrimTest, primitive_name_test)
{
    SpinningMovePrimitive spinning_move_prim =
        SpinningMovePrimitive(0, Point(), AngularVelocity(), 1.0);

    EXPECT_EQ("SpinningMove Primitive", spinning_move_prim.getPrimitiveName());
}

TEST(SpinningMovePrimTest, get_robot_id_test)
{
    unsigned int robot_id = 3U;

    SpinningMovePrimitive spinning_move_prim =
        SpinningMovePrimitive(robot_id, Point(), AngularVelocity(), 1.0);

    EXPECT_EQ(robot_id, spinning_move_prim.getRobotId());
}

TEST(SpinningMovePrimTest, get_angular_vel_test)
{
    const AngularVelocity angular_vel = AngularVelocity::fromRadians(2.5);

    SpinningMovePrimitive spinning_move_prim =
        SpinningMovePrimitive(0, Point(), angular_vel, 1.0);

    EXPECT_EQ(spinning_move_prim.getAngularVelocity(), angular_vel);
}

TEST(SpinningMovePrimTest, get_final_destination_test)
{
    const Point destination = Point(1, -2);

    SpinningMovePrimitive spinning_move_prim =
        SpinningMovePrimitive(0, destination, AngularVelocity(), 1.0);

    EXPECT_EQ(spinning_move_prim.getDestination(), destination);
}

TEST(SpinningMovePrimTest, get_final_speed_test)
{
    const double final_speed = 2.0;

    SpinningMovePrimitive spinning_move_prim =
        SpinningMovePrimitive(0, Point(), AngularVelocity(), final_speed);

    EXPECT_DOUBLE_EQ(spinning_move_prim.getFinalSpeed(), final_speed);
}

TEST(SpinningMovePrimTest, test_equality_operator_primitives_equal)
{
    SpinningMovePrimitive spinning_move_prim =
        SpinningMovePrimitive(0, Point(), AngularVelocity(), 1.0);
    SpinningMovePrimitive spinning_move_prim_other =
        SpinningMovePrimitive(0, Point(), AngularVelocity(), 1.0);

    EXPECT_EQ(spinning_move_prim, spinning_move_prim_other);
}

TEST(SpinningMovePrimTest, test_inequality_operator_with_mismatched_robot_id)
{
    SpinningMovePrimitive spinning_move_prim =
        SpinningMovePrimitive(0, Point(), AngularVelocity(), 1.0);
    SpinningMovePrimitive spinning_move_prim_other =
        SpinningMovePrimitive(6, Point(), AngularVelocity(), 1.0);

    EXPECT_NE(spinning_move_prim, spinning_move_prim_other);
}

TEST(SpinningMovePrimTest, test_inequality_operator_with_mismatched_dest)
{
    SpinningMovePrimitive spinning_move_prim =
        SpinningMovePrimitive(0, Point(), AngularVelocity(), 1.0);
    SpinningMovePrimitive spinning_move_prim_other =
        SpinningMovePrimitive(0, Point(-0.3, 5), AngularVelocity(), 1.0);

    EXPECT_NE(spinning_move_prim, spinning_move_prim_other);
}

TEST(SpinningMovePrimTest, test_inequality_operator_with_mismatched_angular_velocity)
{
    SpinningMovePrimitive spinning_move_prim =
        SpinningMovePrimitive(0, Point(), AngularVelocity(), 1.0);
    SpinningMovePrimitive spinning_move_prim_other =
        SpinningMovePrimitive(0, Point(), AngularVelocity::half(), 1.0);

    EXPECT_NE(spinning_move_prim, spinning_move_prim_other);
}

TEST(SpinningMovePrimTest, test_inequality_operator_with_mismatched_final_speed)
{
    SpinningMovePrimitive spinning_move_prim =
        SpinningMovePrimitive(0, Point(), AngularVelocity(), 1.0);
    SpinningMovePrimitive spinning_move_prim_other =
        SpinningMovePrimitive(0, Point(), AngularVelocity(), 0.5);

    EXPECT_NE(spinning_move_prim, spinning_move_prim_other);
}

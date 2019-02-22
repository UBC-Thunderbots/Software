/**
 * This file contains the unit tests for the MoveSpinPrimitive class
 */

#include <gtest/gtest.h>
#include <string.h>

#include "ai/primitive/grsim_command_primitive_visitor_movespin.h"

TEST(MoveSpinPrimTest, primitive_name_test)
{
    MoveSpinPrimitive movespin_prim = MoveSpinPrimitive(0, Point(), AngularVelocity());

    EXPECT_EQ("MoveSpin Primitive", movespin_prim.getPrimitiveName());
}

TEST(MoveSpinPrimTest, get_robot_id_test)
{
    unsigned int robot_id = 3U;

    MoveSpinPrimitive movespin_prim =
        MoveSpinPrimitive(robot_id, Point(), AngularVelocity());

    EXPECT_EQ(robot_id, movespin_prim.getRobotId());
}

TEST(MoveSpinPrimTest, parameter_array_test)
{
    const Point destination           = Point(1, -2);
    const AngularVelocity angular_vel = AngularVelocity::ofRadians(2.5);
    const unsigned int robot_id       = 1U;

    MoveSpinPrimitive movespin_prim =
        MoveSpinPrimitive(robot_id, destination, angular_vel);

    std::vector<double> param_array = movespin_prim.getParameters();

    EXPECT_DOUBLE_EQ(destination.x(), param_array[0]);
    EXPECT_DOUBLE_EQ(destination.y(), param_array[1]);
    EXPECT_DOUBLE_EQ(angular_vel.toRadians(), param_array[2]);
}

TEST(MoveSpinPrimTest, get_angular_vel_test)
{
    const AngularVelocity angular_vel = AngularVelocity::ofRadians(2.5);

    MoveSpinPrimitive movespin_prim = MoveSpinPrimitive(0, Point(), angular_vel);

    EXPECT_EQ(movespin_prim.getAngularVelocity(), angular_vel);
}

TEST(MoveSpinPrimTest, get_final_destination_test)
{
    const Point destination = Point(1, -2);

    MoveSpinPrimitive movespin_prim =
        MoveSpinPrimitive(0, destination, AngularVelocity());

    EXPECT_EQ(movespin_prim.getDestination(), destination);
}

TEST(MoveSpinPrimTest, get_extra_bit_array_test)
{
    MoveSpinPrimitive movespin_prim = MoveSpinPrimitive(0, Point(), AngularVelocity());

    std::vector<bool> extra_bit_array = movespin_prim.getExtraBits();

    EXPECT_EQ(extra_bit_array, std::vector<bool>());
}

TEST(MoveSpinPrimTest, create_primitive_from_message_test)
{
    const Point destination     = Point(-2, 3);
    const Angle angular_vel     = AngularVelocity::ofRadians(1.45);
    const unsigned int robot_id = 2U;

    MoveSpinPrimitive movespin_prim =
        MoveSpinPrimitive(robot_id, destination, angular_vel);

    thunderbots_msgs::Primitive prim_message = movespin_prim.createMsg();

    MoveSpinPrimitive new_prim = MoveSpinPrimitive(prim_message);

    std::vector<double> parameters = new_prim.getParameters();

    EXPECT_EQ(MoveSpinPrimitive::PRIMITIVE_NAME, new_prim.getPrimitiveName());
    EXPECT_EQ(new_prim, movespin_prim);
}

TEST(MoveSpinPrimTest, test_equality_operator_primitives_equal)
{
    MoveSpinPrimitive movespin_prim = MoveSpinPrimitive(0, Point(), AngularVelocity());
    MoveSpinPrimitive movespin_prim_other =
        MoveSpinPrimitive(0, Point(), AngularVelocity());

    EXPECT_EQ(movespin_prim, movespin_prim_other);
}

TEST(MoveSpinPrimTest, test_inequality_operator_with_mismatched_robot_id)
{
    MoveSpinPrimitive movespin_prim = MoveSpinPrimitive(0, Point(), AngularVelocity());
    MoveSpinPrimitive movespin_prim_other =
        MoveSpinPrimitive(6, Point(), AngularVelocity());

    EXPECT_NE(movespin_prim, movespin_prim_other);
}

TEST(MoveSpinPrimTest, test_inequality_operator_with_mismatched_dest)
{
    MoveSpinPrimitive movespin_prim = MoveSpinPrimitive(0, Point(), AngularVelocity());
    MoveSpinPrimitive movespin_prim_other =
        MoveSpinPrimitive(0, Point(-0.3, 5), AngularVelocity());

    EXPECT_NE(movespin_prim, movespin_prim_other);
}

TEST(MoveSpinPrimTest, test_inequality_operator_with_mismatched_angular_velocity)
{
    MoveSpinPrimitive movespin_prim = MoveSpinPrimitive(0, Point(), AngularVelocity());
    MoveSpinPrimitive movespin_prim_other =
        MoveSpinPrimitive(0, Point(), AngularVelocity::half());

    EXPECT_NE(movespin_prim, movespin_prim_other);
}

/**
 * This file contains the unit tests for the Primitive class
 */

#include "ai/primitive/primitive.h"

#include <gtest/gtest.h>

#include "ai/primitive/direct_velocity_primitive.h"
#include "ai/primitive/directwheels_primitive.h"
#include "ai/primitive/move_primitive.h"
#include "ai/primitive/movespin_primitive.h"

TEST(PrimitiveTest, create_message_from_primitive_test)
{
    const Point destination     = Point(2, -3);
    const Angle final_angle     = Angle::ofRadians(3.55);
    const double final_speed    = 2.22;
    const unsigned int robot_id = 3U;

    MovePrimitive move_prim =
        MovePrimitive(robot_id, destination, final_angle, final_speed);

    thunderbots_msgs::Primitive prim = move_prim.createMsg();

    EXPECT_EQ(prim.primitive_name, "Move Primitive");
    EXPECT_EQ(prim.robot_id, robot_id);
    EXPECT_DOUBLE_EQ(destination.x(), prim.parameters[0]);
    EXPECT_DOUBLE_EQ(destination.y(), prim.parameters[1]);
    EXPECT_DOUBLE_EQ(final_angle.toRadians(), prim.parameters[2]);
    EXPECT_DOUBLE_EQ(final_speed, prim.parameters[3]);
    EXPECT_EQ(move_prim.getExtraBitArray(), std::vector<bool>());
}

TEST(PrimitiveTest, validate_primitive_test)
{
    // TODO: Finish this unit test when exception handling is implemented
    // Issue #133
    MovePrimitive move_prim = MovePrimitive(0, Point(), Angle(), 0.0);

    move_prim.validatePrimitiveMessage(move_prim.createMsg(), "Move Primitive");
}

TEST(PrimitiveTest, create_primitive_from_message_test)
{
    const Point destination     = Point(-3, -3);
    const Angle final_angle     = Angle::ofRadians(4.55);
    const double final_speed    = 4.22;
    const unsigned int robot_id = 0U;

    MovePrimitive move_prim =
        MovePrimitive(robot_id, destination, final_angle, final_speed);

    thunderbots_msgs::Primitive prim_message = move_prim.createMsg();

    std::unique_ptr<Primitive> new_prim = MovePrimitive::createPrimitive(prim_message);

    std::vector<double> param_array = new_prim->getParameterArray();

    EXPECT_EQ("Move Primitive", new_prim->getPrimitiveName());
    EXPECT_EQ(robot_id, new_prim->getRobotId());
    EXPECT_DOUBLE_EQ(destination.x(), param_array[0]);
    EXPECT_DOUBLE_EQ(destination.y(), param_array[1]);
    EXPECT_DOUBLE_EQ(final_angle.toRadians(), param_array[2]);
    EXPECT_DOUBLE_EQ(final_speed, param_array[3]);
    EXPECT_EQ(new_prim->getExtraBitArray(), std::vector<bool>());
}

TEST(PrimitiveTest, creat_MoveSpinPrimitive_from_message_test)
{
    const Point destination     = Point(-1, 4);
    const Angle angular_vel     = AngularVelocity::ofRadians(0.54);
    const unsigned int robot_id = 2U;

    MoveSpinPrimitive movespin_prim =
        MoveSpinPrimitive(robot_id, destination, angular_vel);

    thunderbots_msgs::Primitive prim_message = movespin_prim.createMsg();

    MoveSpinPrimitive new_prim = MoveSpinPrimitive(prim_message);

    std::vector<double> parameters = new_prim.getParameterArray();

    EXPECT_EQ("MoveSpin Primitive", new_prim.getPrimitiveName());
    EXPECT_EQ(robot_id, new_prim.getRobotId());
    EXPECT_DOUBLE_EQ(destination.x(), parameters[0]);
    EXPECT_DOUBLE_EQ(destination.y(), parameters[1]);
    EXPECT_DOUBLE_EQ(angular_vel.toRadians(), parameters[2]);
    EXPECT_EQ(movespin_prim.getExtraBitArray(), std::vector<bool>());
}

TEST(PrimitiveTest, creat_DirectWheelsPrimitive_from_message_test)
{
    const signed int wheel0_power = 2;
    const signed int wheel1_power = 4;
    const signed int wheel2_power = 6;
    const signed int wheel3_power = 8;
    const double dribbler_rpm     = 49.6;
    const unsigned int robot_id   = 3U;

    DirectWheelsPrimitive directwheels_prim = DirectWheelsPrimitive(
        robot_id, wheel0_power, wheel1_power, wheel2_power, wheel3_power, dribbler_rpm);

    thunderbots_msgs::Primitive prim_message = directwheels_prim.createMsg();

    DirectWheelsPrimitive new_prim = DirectWheelsPrimitive(prim_message);

    std::vector<double> parameters = new_prim.getParameterArray();

    EXPECT_EQ("DirectWheels Primitive", new_prim.getPrimitiveName());
    EXPECT_EQ(robot_id, new_prim.getRobotId());
    EXPECT_DOUBLE_EQ(wheel0_power, parameters[0]);
    EXPECT_DOUBLE_EQ(wheel1_power, parameters[1]);
    EXPECT_DOUBLE_EQ(wheel2_power, parameters[2]);
    EXPECT_DOUBLE_EQ(wheel3_power, parameters[3]);
    EXPECT_DOUBLE_EQ(dribbler_rpm, parameters[4]);
    EXPECT_EQ(directwheels_prim.getExtraBitArray(), std::vector<bool>());
}

TEST(PrimitiveTest, creat_DirectVelocityPrimitive_from_message_test)
{
    const unsigned int robot_id                  = 1U;
    const double x_velocity                      = 2.78;
    const double y_velocity                      = -1.414;
    const double angular_velocity                = -0.98;
    const double dribbler_rpm                    = 9.047;
    DirectVelocityPrimitive direct_velocity_prim = DirectVelocityPrimitive(
        robot_id, x_velocity, y_velocity, angular_velocity, dribbler_rpm);

    thunderbots_msgs::Primitive prim_message = direct_velocity_prim.createMsg();
    std::unique_ptr<Primitive> new_prim =
        DirectVelocityPrimitive::createPrimitive(prim_message);
    std::vector<double> params = new_prim->getParameterArray();

    EXPECT_EQ("Direct Velocity Primitive", new_prim->getPrimitiveName());
    EXPECT_EQ(robot_id, new_prim->getRobotId());
    EXPECT_DOUBLE_EQ(x_velocity, params[0]);
    EXPECT_DOUBLE_EQ(y_velocity, params[1]);
    EXPECT_DOUBLE_EQ(angular_velocity, params[2]);
    EXPECT_DOUBLE_EQ(dribbler_rpm, params[3]);
    EXPECT_EQ(std::vector<bool>(), new_prim->getExtraBitArray());
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

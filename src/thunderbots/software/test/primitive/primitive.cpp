/**
 * This file contains the unit tests for the Primitive class
 */

#include "ai/primitive/primitive.h"

#include <gtest/gtest.h>

#include "ai/primitive/catch_primitive.h"
#include "ai/primitive/chip_primitive.h"
#include "ai/primitive/direct_velocity_primitive.h"
#include "ai/primitive/kick_primitive.h"
#include "ai/primitive/move_primitive.h"
#include "ai/primitive/pivot_primitive.h"
#include "ai/primitive/stop_primitive.h"

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
    EXPECT_EQ(move_prim.getExtraBits(), std::vector<bool>());
}

TEST(PrimitiveTest, validate_primitive_test)
{
    // TODO: Finish this unit test when exception handling is implemented
    // Issue #133
    MovePrimitive move_prim = MovePrimitive(0, Point(), Angle(), 0.0);

    move_prim.validatePrimitiveMessage(move_prim.createMsg(), "Move Primitive");
}

// Test that we can correctly translate a MovePrimitive like:
// `Primitive` -> `ROS Message` -> `Primitive` and get back the same primitive we
// started with
TEST(PrimitiveTest, convert_MovePrimitive_to_message_and_back_to_MovePrimitive)
{
    const Point destination     = Point(-3, -3);
    const Angle final_angle     = Angle::ofRadians(4.55);
    const double final_speed    = 4.22;
    const unsigned int robot_id = 0U;

    MovePrimitive move_prim =
        MovePrimitive(robot_id, destination, final_angle, final_speed);

    thunderbots_msgs::Primitive prim_message = move_prim.createMsg();

    std::unique_ptr<Primitive> new_prim = Primitive::createPrimitive(prim_message);

    // Since we have a `Primitive` and NOT a `MovePrimitive`, we have to check that the
    // values are correct by getting them from the generic parameter array
    std::vector<double> params = new_prim->getParameters();

    EXPECT_EQ("Move Primitive", new_prim->getPrimitiveName());
    EXPECT_EQ(robot_id, new_prim->getRobotId());
    EXPECT_DOUBLE_EQ(destination.x(), params[0]);
    EXPECT_DOUBLE_EQ(destination.y(), params[1]);
    EXPECT_DOUBLE_EQ(final_angle.toRadians(), params[2]);
    EXPECT_DOUBLE_EQ(final_speed, params[3]);
    EXPECT_EQ(new_prim->getExtraBits(), std::vector<bool>());
}

// Test that we can correctly translate a ChipPrimitive like:
// `Primitive` -> `ROS Message` -> `Primitive` and get back the same primitive we
// started with
TEST(PrimitiveTest, convert_ChipPrimitive_to_message_and_back_to_ChipPrimitive)
{
    const unsigned int robot_id       = 0U;
    const Point chip_origin           = Point(-3, -2.5);
    const Angle chip_direction        = Angle::ofRadians(2.37);
    const double chip_distance_meters = 4.2;

    ChipPrimitive chip_prim =
        ChipPrimitive(robot_id, chip_origin, chip_direction, chip_distance_meters);

    thunderbots_msgs::Primitive prim_message = chip_prim.createMsg();

    std::unique_ptr<Primitive> new_prim = ChipPrimitive::createPrimitive(prim_message);

    // Since we have a `Primitive` and NOT a `MovePrimitive`, we have to check that the
    // values are correct by getting them from the generic parameter array
    std::vector<double> params = new_prim->getParameters();

    EXPECT_EQ("Chip Primitive", new_prim->getPrimitiveName());
    EXPECT_EQ(robot_id, new_prim->getRobotId());
    EXPECT_DOUBLE_EQ(chip_origin.x(), params[0]);
    EXPECT_DOUBLE_EQ(chip_origin.y(), params[1]);
    EXPECT_DOUBLE_EQ(chip_direction.toRadians(), params[2]);
    EXPECT_DOUBLE_EQ(chip_distance_meters, params[3]);
    EXPECT_EQ(new_prim->getExtraBits(), std::vector<bool>());
}

// Test that we can correctly translate a KickPrimitive like:
// `Primitive` -> `ROS Message` -> `Primitive` and get back the same primitive we
// started with
TEST(PrimitiveTest, convert_KickPrimitive_to_message_and_back_to_KickPrimitive)
{
    const unsigned int robot_id       = 0U;
    const Point kick_origin           = Point(-3, -2.5);
    const Angle kick_direction        = Angle::ofRadians(2.37);
    const double kick_distance_meters = 4.2;

    KickPrimitive kick_prim =
        KickPrimitive(robot_id, kick_origin, kick_direction, kick_distance_meters);

    thunderbots_msgs::Primitive prim_message = kick_prim.createMsg();

    std::unique_ptr<Primitive> new_prim = KickPrimitive::createPrimitive(prim_message);

    // Since we have a `Primitive` and NOT a `MovePrimitive`, we have to check that the
    // values are correct by getting them from the generic parameter array
    std::vector<double> params = new_prim->getParameters();

    EXPECT_EQ("Kick Primitive", new_prim->getPrimitiveName());
    EXPECT_EQ(robot_id, new_prim->getRobotId());
    EXPECT_DOUBLE_EQ(kick_origin.x(), params[0]);
    EXPECT_DOUBLE_EQ(kick_origin.y(), params[1]);
    EXPECT_DOUBLE_EQ(kick_direction.toRadians(), params[2]);
    EXPECT_DOUBLE_EQ(kick_distance_meters, params[3]);
    EXPECT_EQ(new_prim->getExtraBits(), std::vector<bool>());
}

TEST(PrimitiveTest,
     convert_DirectVelocityPrimitive_to_message_and_back_to_DirectVelocityPrimitive)
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
    std::vector<double> params = new_prim->getParameters();

    EXPECT_EQ("Direct Velocity Primitive", new_prim->getPrimitiveName());
    EXPECT_EQ(robot_id, new_prim->getRobotId());
    EXPECT_DOUBLE_EQ(x_velocity, params[0]);
    EXPECT_DOUBLE_EQ(y_velocity, params[1]);
    EXPECT_DOUBLE_EQ(angular_velocity, params[2]);
    EXPECT_DOUBLE_EQ(dribbler_rpm, params[3]);
    EXPECT_EQ(std::vector<bool>(), new_prim->getExtraBits());
}

TEST(PrimitiveTest, convert_CatchPrimitive_to_message_and_back_to_CatchPrimitive)
{
    const unsigned int robot_id        = 1U;
    const double velocity              = 7.0;
    const double dribbler_rpm          = 60;
    const double ball_intercept_margin = 0.5;
    CatchPrimitive catch_prim(robot_id, velocity, dribbler_rpm, ball_intercept_margin);

    thunderbots_msgs::Primitive prim_message = catch_prim.createMsg();
    std::unique_ptr<Primitive> new_prim = CatchPrimitive::createPrimitive(prim_message);
    std::vector<double> params          = new_prim->getParameters();

    EXPECT_EQ("Catch Primitive", new_prim->getPrimitiveName());
    EXPECT_EQ(robot_id, new_prim->getRobotId());
    EXPECT_DOUBLE_EQ(velocity, params[0]);
    EXPECT_DOUBLE_EQ(dribbler_rpm, params[1]);
    EXPECT_DOUBLE_EQ(ball_intercept_margin, params[2]);
    EXPECT_EQ(std::vector<bool>(), new_prim->getExtraBits());
}

// Test that we can correctly translate a PivotPrimitive like:
// `Primitive` -> `ROS Message` -> `Primitive` and get back the same primitive we
// started with
TEST(PivotPrimTest, convert_PivotPrimitive_to_message_and_back_to_PivotPrimitive)
{
    const unsigned int robot_id = 2U;
    const Point pivot_point     = Point(2, -1);
    const Angle final_angle     = Angle::ofRadians(2.56);
    const double pivot_radius   = .78;

    PivotPrimitive pivot_prim =
        PivotPrimitive(robot_id, pivot_point, final_angle, pivot_radius);

    thunderbots_msgs::Primitive prim_msg = pivot_prim.createMsg();
    std::unique_ptr<Primitive> new_prim  = PivotPrimitive::createPrimitive(prim_msg);
    std::vector<double> parameters       = new_prim->getParameters();

    EXPECT_EQ("Pivot Primitive", new_prim->getPrimitiveName());
    EXPECT_EQ(robot_id, new_prim->getRobotId());
    EXPECT_DOUBLE_EQ(pivot_point.x(), parameters[0]);
    EXPECT_DOUBLE_EQ(pivot_point.y(), parameters[1]);
    EXPECT_DOUBLE_EQ(final_angle.toRadians(), parameters[2]);
    EXPECT_DOUBLE_EQ(pivot_radius, parameters[3]);
    EXPECT_EQ(std::vector<bool>(), new_prim->getExtraBits());
}

// Test that we can correctly translate a StopPrimitive like:
// `Primitive` -> `ROS Message` -> `Primitive` and get back the same primitive we
// started with
TEST(StopPrimTest, convert_StopPrimitive_to_message_and_back_to_StopPrimitive)
{
    const unsigned int robot_id = 2U;
    const bool coast            = false;

    StopPrimitive stop_prim                  = StopPrimitive(robot_id, coast);
    thunderbots_msgs::Primitive prim_message = stop_prim.createMsg();

    std::unique_ptr<Primitive> new_prim = StopPrimitive::createPrimitive(prim_message);

    // Since we have a `Primitive` and NOT a `StopPrimitive`, we have to check that the
    // values are correct by getting them from the generic extra_bits array
    // we use the extra_bits array instead of the parameters array because `StopPrimitive`
    // has only one variable, coast
    std::vector<bool> extra_bits = new_prim->getExtraBits();

    EXPECT_EQ("Stop Primitive", new_prim->getPrimitiveName());
    EXPECT_EQ(robot_id, new_prim->getRobotId());
    EXPECT_EQ(coast, extra_bits[0]);
    EXPECT_EQ(new_prim->getParameters(), std::vector<double>());
    EXPECT_EQ(std::vector<double>(), new_prim->getParameters());
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

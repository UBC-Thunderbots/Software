/**
 * This file contains the unit tests for the grsim command implementation
 * of the DribblePrimitive class
 */

#include "software/primitive/dribble_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

#include "shared/constants.h"
#include "software/backend/output/grsim/command_primitive_visitor/grsim_command_primitive_visitor.h"
#include "software/world/robot.h"

using MotionControllerCommand =
    std::variant<MotionController::PositionCommand, MotionController::VelocityCommand>;

TEST(GrsimCommandPrimitiveVisitorTest, visit_dribble_primitive_kick_allowed)
{
    DribblePrimitive dribble_primitive =
        DribblePrimitive(0, Point(-0.2, 1), AngularVelocity::fromDegrees(75), 50.0, true);

    Robot test_robot = Robot(1, Point(0, 0), Vector(1, 2), Angle::zero(),
                             AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball test_ball   = Ball(Point(1, 0), Vector(-1, -1), Timestamp::fromSeconds(0));

    auto grsimCommandPrimitiveVisitor =
        GrsimCommandPrimitiveVisitor(test_robot, test_ball);
    dribble_primitive.accept(grsimCommandPrimitiveVisitor);

    auto motion_controller_command = std::get<MotionController::PositionCommand>(
        grsimCommandPrimitiveVisitor.getMotionControllerCommand());

    EXPECT_EQ(motion_controller_command.global_destination, Point(-0.2, 1));
    EXPECT_EQ(motion_controller_command.final_orientation, Angle::fromDegrees(75));
    EXPECT_EQ(motion_controller_command.kick_speed_meters_per_second, 0);
    EXPECT_TRUE(motion_controller_command.dribbler_on);
    EXPECT_FALSE(motion_controller_command.chip_instead_of_kick);
}

TEST(GrsimCommandPrimitiveVisitorTest, visit_dribble_primitive_kick_not_allowed)
{
    DribblePrimitive dribble_primitive = DribblePrimitive(
        0, Point(-0.2, 1), AngularVelocity::fromDegrees(75), 50.0, false);

    Robot test_robot = Robot(1, Point(0, 0), Vector(1, 2), Angle::zero(),
                             AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball test_ball   = Ball(Point(1, 0), Vector(-1, -1), Timestamp::fromSeconds(0));

    auto grsimCommandPrimitiveVisitor =
        GrsimCommandPrimitiveVisitor(test_robot, test_ball);
    dribble_primitive.accept(grsimCommandPrimitiveVisitor);

    auto motion_controller_command = std::get<MotionController::PositionCommand>(
        grsimCommandPrimitiveVisitor.getMotionControllerCommand());

    EXPECT_EQ(motion_controller_command.global_destination, Point(-0.2, 1));
    EXPECT_EQ(motion_controller_command.final_orientation, Angle::fromDegrees(75));
    EXPECT_EQ(motion_controller_command.kick_speed_meters_per_second, 0.0);
    EXPECT_TRUE(motion_controller_command.dribbler_on);
    EXPECT_FALSE(motion_controller_command.chip_instead_of_kick);
}

/**
 * This file contains the unit tests for the grsim command implementation
 * of the MoveSpinPrimitive class
 */

#include "software/primitive/movespin_primitive.h"

#include <gtest/gtest.h>
#include <string.h>

#include "software/backend/output/grsim/command_primitive_visitor/grsim_command_primitive_visitor.h"
#include "software/world/robot.h"

TEST(GrsimCommandPrimitiveVisitorTest, visit_move_spin_primitive)
{
    MoveSpinPrimitive* move_spin_primitive =
        new MoveSpinPrimitive(1, Point(3, -1), AngularVelocity::fromDegrees(50), 1.0);

    Robot* test_robot = new Robot(1, Point(0, 0), Vector(1, 2), Angle::zero(),
                                  AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Ball* test_ball   = new Ball(Point(1, 0), Vector(-1, -1), Timestamp::fromSeconds(0));

    auto* grsimCommandPrimitiveVisitor =
        new GrsimCommandPrimitiveVisitor(*test_robot, *test_ball);
    move_spin_primitive->accept(*grsimCommandPrimitiveVisitor);

    auto motion_controller_command = std::get<MotionController::PositionCommand>(
        grsimCommandPrimitiveVisitor->getMotionControllerCommand());

    EXPECT_EQ(motion_controller_command.global_destination, Point(3, -1));
    EXPECT_EQ(motion_controller_command.final_orientation, Angle::fromDegrees(45));
    EXPECT_DOUBLE_EQ(motion_controller_command.final_speed_at_destination, 1.0);
    EXPECT_EQ(motion_controller_command.kick_speed_meters_per_second, 0.0);
    EXPECT_FALSE(motion_controller_command.dribbler_on);
    EXPECT_FALSE(motion_controller_command.chip_instead_of_kick);
}

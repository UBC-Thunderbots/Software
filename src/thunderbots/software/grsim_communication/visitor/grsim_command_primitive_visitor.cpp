#include "grsim_communication/visitor/grsim_command_primitive_visitor.h"

#include "ai/primitive/catch_primitive.h"
#include "ai/primitive/chip_primitive.h"
#include "ai/primitive/direct_velocity_primitive.h"
#include "ai/primitive/kick_primitive.h"
#include "ai/primitive/move_primitive.h"
#include "ai/primitive/pivot_primitive.h"
#include "ai/primitive/stop_primitive.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "util/logger/init.h"

GrsimCommandPrimitiveVisitor::GrsimCommandPrimitiveVisitor(const Robot &robot)
    : robot(robot)
{
}

void GrsimCommandPrimitiveVisitor::visit(const CatchPrimitive &catch_primitive)
{
    // TODO: https://github.com/UBC-Thunderbots/Software/issues/134
}

void GrsimCommandPrimitiveVisitor::visit(const ChipPrimitive &chip_primitive)
{
    // Determine if robot is at least 1 robot radius from the given location
    if ((chip_primitive.getChipOrigin() - robot.position()).len() >= 1)
    {
        // Travel in straight line from current position to given location
        motion_controller_command = MotionController::MotionControllerCommand(
            chip_primitive.getChipOrigin(), chip_primitive.getChipDirection(), 0.0,
            chip_primitive.getChipDistance(), true, false);

        // Move in a straight line directly behind the robot
        Point destBehind;
        destBehind = chip_primitive.getChipOrigin();
        destBehind = Point(destBehind.x(), destBehind.y());
        destBehind.set(destBehind.x() - 0.5, destBehind.y() - 0.5);

        motion_controller_command = MotionController::MotionControllerCommand(
            destBehind, chip_primitive.getChipDirection(), 0.0,
            chip_primitive.getChipDistance(), true, false);

        // Move forward to chip
        Point destForward;
        destForward = chip_primitive.getChipOrigin();
        destForward = Point(destForward.x(), destForward.y());
        destForward.set(destForward.x() + 0.8, destForward.y() + 0.8);

        motion_controller_command = MotionController::MotionControllerCommand(
            destForward, chip_primitive.getChipDirection(), 0.0,
            chip_primitive.getChipDistance(), true, false);
    }
}

void GrsimCommandPrimitiveVisitor::visit(
    const DirectVelocityPrimitive &direct_velocity_primitive)
{
    // TODO: https://github.com/UBC-Thunderbots/Software/issues/97
}

void GrsimCommandPrimitiveVisitor::visit(
    const DirectWheelsPrimitive &direct_wheels_primitive)
{
    // TODO: https://github.com/UBC-Thunderbots/Software/issues/98
}

void GrsimCommandPrimitiveVisitor::visit(const KickPrimitive &kick_primitive)
{
    // Determine if robot is at least 1 robot radius from the given location
    if ((kick_primitive.getKickOrigin() - robot.position()).len() >= 1)
    {
        // Travel in straight line from current position to given location
        motion_controller_command = MotionController::MotionControllerCommand(
            kick_primitive.getKickOrigin(), kick_primitive.getKickDirection(), 0.0,
            kick_primitive.getKickSpeed(), false, false);

        // Move in a straight line directly behind the robot
        Point destBehind;
        destBehind = kick_primitive.getKickOrigin();
        destBehind = Point(destBehind.x(), destBehind.y());
        destBehind.set(destBehind.x() - 0.5, destBehind.y() - 0.5);

        motion_controller_command = MotionController::MotionControllerCommand(
            destBehind, kick_primitive.getKickDirection(), 0.0,
            kick_primitive.getKickSpeed(), false, false);

        // Move forward to kick
        Point destForward;
        destForward = kick_primitive.getKickOrigin();
        destForward = Point(destForward.x(), destForward.y());
        destForward.set(destForward.x() + 0.8, destForward.y() + 0.8);

        motion_controller_command = MotionController::MotionControllerCommand(
            destForward, kick_primitive.getKickDirection(), 0.0,
            kick_primitive.getKickSpeed(), false, false);
    }
}

void GrsimCommandPrimitiveVisitor::visit(const MovePrimitive &move_primitive)
{
    motion_controller_command = MotionController::MotionControllerCommand(
        move_primitive.getDestination(), move_primitive.getFinalAngle(),
        move_primitive.getFinalSpeed(), 0.0, false, false);
}

void GrsimCommandPrimitiveVisitor::visit(const MoveSpinPrimitive &move_spin_primitive)
{
    // TODO: https://github.com/UBC-Thunderbots/Software/issues/95
}

void GrsimCommandPrimitiveVisitor::visit(const PivotPrimitive &pivot_primitive)
{
    // TODO: https://github.com/UBC-Thunderbots/Software/issues/94
}

void GrsimCommandPrimitiveVisitor::visit(const StopPrimitive &stop_primitive)
{
    // TODO: https://github.com/UBC-Thunderbots/Software/issues/94
}

MotionController::MotionControllerCommand
GrsimCommandPrimitiveVisitor::getMotionControllerCommand()
{
    return motion_controller_command;
}

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
    // TODO: https://github.com/UBC-Thunderbots/Software/issues/93
}

void GrsimCommandPrimitiveVisitor::visit(
    const DirectVelocityPrimitive &direct_velocity_primitive)
{
    // TODO: https://github.com/UBC-Thunderbots/Software/issues/97
}

void GrsimCommandPrimitiveVisitor::visit(const KickPrimitive &kick_primitive)
{
    // TODO: https://github.com/UBC-Thunderbots/Software/issues/93
}

void GrsimCommandPrimitiveVisitor::visit(const MovePrimitive &move_primitive)
{
    motion_controller_command = MotionController::MotionControllerCommand(
        move_primitive.getDestination(), move_primitive.getFinalAngle(),
        move_primitive.getFinalSpeed(), 0.0, false, false);
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

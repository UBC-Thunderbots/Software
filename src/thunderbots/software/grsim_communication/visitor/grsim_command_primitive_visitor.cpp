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
    // get current robot position and orientation(angle)
    Point robot_position    = robot.position();
    Angle robot_orientation = robot.orientation();

    // create linear velocity vector from direct velocity primitive
    Vector linear_velocity_in_robot_coordinates =
        Vector(direct_velocity_primitive.getXVelocity(),
               direct_velocity_primitive.getXVelocity());

    // transfer velocity into global coordinate by rotating the vector in robot
    // coordinates by the angle of robot
    Vector linear_velocity_in_global_coordinates =
        linear_velocity_in_robot_coordinates.rotate(robot.orientation());

    // TODO final reasonable scalars to multiply linear and angular velocity to get
    // reasonable speed final destination is the parameter that can control the robot to
    // move in the direction of velocity vector from current robot position
    Vector final_destination = linear_velocity_in_global_coordinates + robot_position;
    // final orientation is the parameter that can control the robot to rotate in the
    // direction of angular velocity from current robot orientation
    Angle final_orientation =
        robot_orientation +
        Angle::ofRadians(direct_velocity_primitive.getAngularVelocity());

    motion_controller_command = MotionController::MotionControllerCommand(
        final_destination, final_orientation, linear_velocity_in_robot_coordinates.len(),
        0.0, false, direct_velocity_primitive.getDribblerRpm() > 0);
}

void GrsimCommandPrimitiveVisitor::visit(
    const DirectWheelsPrimitive &direct_wheels_primitive)
{
    // TODO: https://github.com/UBC-Thunderbots/Software/issues/98
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

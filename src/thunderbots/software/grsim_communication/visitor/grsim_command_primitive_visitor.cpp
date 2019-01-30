#include "grsim_communication/visitor/grsim_command_primitive_visitor.h"

#include "ai/primitive/catch_primitive.h"
#include "ai/primitive/chip_primitive.h"
#include "ai/primitive/direct_velocity_primitive.h"
#include "ai/primitive/kick_primitive.h"
#include "ai/primitive/move_primitive.h"
#include "ai/primitive/pivot_primitive.h"
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
    double x_velocity = direct_velocity_primitive.getXVelocity();
    double y_velocity = direct_velocity_primitive.getYVelocity();
    double angular_velocity = direct_velocity_primitive.getAngularVelocity();
    double dribbler_rpm =direct_velocity_primitive.getDribblerRpm();

    Point robot_position = robot.position();
    Angle robot_orientation = robot.orientation();

    Vector linear_velocity_in_robot_coordinates  = Vector(x_velocity, y_velocity);
    Vector linear_velocity_in_global_coordinates = linear_velocity_in_robot_coordinates.rotate(robot.orientation());

    Vector global_destination = linear_velocity_in_global_coordinates - robot_position;
    Angle final_orientation = robot_orientation+robot_orientation*(angular_velocity/60);

    motion_controller_command = MotionController::MotionControllerCommand(
            global_destination, final_orientation,
            linear_velocity_in_robot_coordinates.len() , 0.0, false, dribbler_rpm>0);
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

MotionController::MotionControllerCommand
GrsimCommandPrimitiveVisitor::getMotionControllerCommand()
{
    return motion_controller_command;
}

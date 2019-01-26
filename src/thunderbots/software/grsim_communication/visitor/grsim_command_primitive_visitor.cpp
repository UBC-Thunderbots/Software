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
#include "shared/constants.h"  // move to header file?
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

    // use v_f^2 = v_i^2 + 2ad; v_f = 0 m/s; v_i = ROBOT_MAX_SPEED_METERS_PER_SECOND
    double dist_stop_from_max_speed_metres =
        ROBOT_MAX_SPEED_METERS_PER_SECOND * ROBOT_MAX_SPEED_METERS_PER_SECOND / 2 /
        ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;

    // assume grSim is running at 60Hz TODO: this should go somewhere else
    double assumed_t_step_seconds = 1 / 60;

    // the target point for the pivot
    Point pivot_dest =
        pivot_primitive.getPivotPoint() + Point(pivot_primitive.getPivotRadius(), 0)
                                              .rotate(pivot_primitive.getFinalAngle());

    //    double distance_to_travel = 0.4; // get the calc for that
    //
    //    Point next_step_midpoint = Point(); // this temporary
    //
    //    Point destination = Point(); // get from distance travelled and direction
    //
    //    Angle final_orientation = Angle(); // calc based on end relative to centre
    //
    //    double final_speed = .5; // based on current speed + accel


    double final_speed =
        (robot.position() - pivot_dest).len() > dist_stop_from_max_speed_metres
            ? ROBOT_MAX_SPEED_METERS_PER_SECOND
            : 0;


    Point expected_destination;  // todo

    // the distance we expect to travel
    double expected_distance = (expected_destination - robot.position()).len();

    // the unit vector representing the direction the robot will travel
    Vector travel_direction = (expected_destination - robot.position()).norm();

    // a point twice the distance from robot to expected destination in same direction to
    // absorb errors in calculations and prevent decelerating and accelerating in a
    // certain timestep
    Point point_past_expected_dest =
        expected_destination + expected_distance * travel_direction;

    Angle final_orientation =
        (pivot_primitive.getPivotPoint() - expected_destination).orientation();


    motion_controller_command = MotionController::MotionControllerCommand(
        point_past_expected_dest, final_orientation, final_speed, 0.0, false, false);
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

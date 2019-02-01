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
#include "geom/util.h"
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

    // use v_f^2 = v_i^2 + 2ad; v_f = 0 m/s; v_i = ROBOT_MAX_SPEED_METERS_PER_SECOND
    double dist_stop_from_max_speed_metres =
        ROBOT_MAX_SPEED_METERS_PER_SECOND * ROBOT_MAX_SPEED_METERS_PER_SECOND / 2 /
        ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;

    // when the robot is this angle away from end location, should be slowing down
    Angle stop_angle = Angle::ofRadians(dist_stop_from_max_speed_metres /
                                        pivot_primitive.getPivotRadius());

    // assume grSim is running at 60Hz TODO: this should go somewhere else
    double assumed_t_step_seconds = 1 / 60;

    // the target point for the pivot
    Point pivot_dest =
        pivot_primitive.getPivotPoint() + Point(pivot_primitive.getPivotRadius(), 0)
                                              .rotate(pivot_primitive.getFinalAngle());

    double final_speed =
        (robot.position() - pivot_dest).len() > dist_stop_from_max_speed_metres
            ? ROBOT_MAX_SPEED_METERS_PER_SECOND
            : 0;

    Vector pivot_point_to_robot = robot.position() - pivot_primitive.getPivotPoint();

    Vector pivot_point_to_pivot_circle =
        pivot_point_to_robot.norm(pivot_primitive.getPivotRadius());

    // expect robot to not be on circle; define an error vector from the robot's position
    // to the point on the circle with the same angle
    Vector radial_error = pivot_point_to_pivot_circle - pivot_point_to_robot;

    Vector tangent_to_circle_CCW =
        pivot_point_to_pivot_circle.rotate(Angle::ofDegrees(90));

    // if the shortest direction is a clockwise rotation, invert this vector
    Angle rotation_remaining =
        ((pivot_dest - pivot_primitive.getPivotPoint()).orientation() -
         pivot_point_to_pivot_circle.orientation())
            .clamp();
    Vector tangent_correct_direction = (rotation_remaining < Angle().zero())
                                           ? -1 * tangent_to_circle_CCW
                                           : tangent_to_circle_CCW;

    // unit vector representing direction robot will travel
    Vector travel_direction = (tangent_correct_direction + radial_error).norm();

    // how far we expect to travel
    // assume current speed, time interval grSim is using TODO; verify latter is fine
    double robot_speed = robot.velocity().len();
    double expected_distance =
        assumed_t_step_seconds *
                    (robot_speed < 0.05 * ROBOT_MAX_SPEED_METERS_PER_SECOND) &&
                (rotation_remaining > stop_angle)
            ? robot_speed + assumed_t_step_seconds *
                                ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
            : robot_speed;


    Point expected_destination = expected_distance * travel_direction + robot.position();

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

#include "grsim_command_primitive_visitor.h"

#include <shared/constants.h>

#include "ai/primitive/catch_primitive.h"
#include "ai/primitive/chip_primitive.h"
#include "ai/primitive/direct_velocity_primitive.h"
#include "ai/primitive/direct_wheels_primitive.h"
#include "ai/primitive/dribble_primitive.h"
#include "ai/primitive/kick_primitive.h"
#include "ai/primitive/move_primitive.h"
#include "ai/primitive/movespin_primitive.h"
#include "ai/primitive/pivot_primitive.h"
#include "ai/primitive/stop_primitive.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "geom/util.h"
#include "shared/constants.h"
#include "util/logger/init.h"

GrsimCommandPrimitiveVisitor::GrsimCommandPrimitiveVisitor(const Robot &robot,
                                                           const Ball &ball)
    : robot(robot), ball(ball)
{
}

void GrsimCommandPrimitiveVisitor::visit(const CatchPrimitive &catch_primitive)
{
    // Calculate the theoretical catch point based on the robots's distance to the ball
    // and how fast the robot is moving.
    double distanceToBall = (ball.position() - robot.position()).len();
    Point finalDest;
    if (robot.velocity().len() != 0)
    {
        finalDest = ball.estimatePositionAtFutureTime(
            Duration::fromSeconds((int)fabs(distanceToBall / robot.velocity().len())));
        finalDest = Point(finalDest.x(), finalDest.y());
    }
    else
    {
        // If Robot is not moving, estimate position based on a standard velocity of 1
        finalDest = ball.estimatePositionAtFutureTime(
            Duration::fromSeconds((int)fabs(distanceToBall / 1)));
        finalDest = Point(finalDest.x(), finalDest.y());
    }

    // Get unit vectors in the direction the ball is moving.
    // This allows the interceptor margin to be applied in the correct direction.
    double ballDirX = 0, ballDirY = 0;
    if (ball.velocity().x() != 0)
    {
        ballDirX = ball.velocity().x() / abs(ball.velocity().x());
    }
    if (ball.velocity().y() != 0)
    {
        ballDirY = ball.velocity().y() / abs(ball.velocity().y());
    }

    // Robot should be facing in the opposite direction the ball in moving to have ball
    // hit its dribbler.
    Angle robotDirection =
        Angle::ofRadians(std::atan2(ball.velocity().y(), ball.velocity().x())) +
        Angle::half();

    // If ball is far enough way from robot, add extra margin of error to ensure robot
    // gets in line with the ball correctly. In addition, if the robot is not yet facing
    // the ball move away to give it time to adjust.
    double interceptonMargin = STANDARD_INTERCEPT_MARGIN * catch_primitive.getMargin();
    if (distanceToBall > interceptonMargin || robot.orientation() == robotDirection)
    {
        finalDest.set(finalDest.x() + interceptonMargin * ballDirX,
                      finalDest.y() + interceptonMargin * ballDirY);
    }

    motion_controller_command = MotionController::PositionCommand(
        finalDest, robotDirection, catch_primitive.getVelocity(), 0.0, false,
        catch_primitive.getDribblerSpeed() > 0);
}

void GrsimCommandPrimitiveVisitor::visit(const ChipPrimitive &chip_primitive)
{
    Point chip_origin    = chip_primitive.getChipOrigin();
    Angle chip_direction = chip_primitive.getChipDirection();

    double final_speed_at_destination = 0.0;

    Point point_behind = chip_origin + Vector(Point::createFromAngle(chip_direction).x(),
                                              Point::createFromAngle(chip_direction).y());

    Point closest_point_to_line =
        closestPointOnLine(robot.position(), chip_origin, point_behind);

    // If current robot position is in line with the shot (i.e. less than two robot radius
    // within the line in the direction of the shot), can go for the shot
    if (offsetToLine(chip_origin, point_behind, robot.position()) <=
        2 * ROBOT_MAX_RADIUS_METERS)
    {
        motion_controller_command = MotionController::PositionCommand(
            chip_origin, chip_direction, final_speed_at_destination,
            chip_primitive.getChipDistance(), true, false);
    }
    else
    {
        // If robot is not in line, move to the closest point on the line from the current
        // position
        motion_controller_command = MotionController::PositionCommand(
            closest_point_to_line, chip_direction, 0.0, chip_primitive.getChipDistance(),
            true, false);
    }
}

void GrsimCommandPrimitiveVisitor::visit(
    const DirectVelocityPrimitive &direct_velocity_primitive)
{
    // TODO: https://github.com/UBC-Thunderbots/Software/issues/286

    // get current robot position and orientation(angle)
    Point robot_position    = robot.position();
    Angle robot_orientation = robot.orientation();

    // create linear velocity vector from direct velocity primitive
    Vector linear_velocity_in_robot_coordinates =
        Vector(direct_velocity_primitive.getXVelocity(),
               direct_velocity_primitive.getYVelocity());

    // transfer velocity into global coordinate by rotating the vector in robot
    // coordinates by the angle of robot
    Vector linear_velocity_in_global_coordinates =
        linear_velocity_in_robot_coordinates.rotate(robot.orientation());

    // final destination is the parameter that can control the robot to
    // move in the direction of velocity vector from current robot position
    Vector final_destination = linear_velocity_in_global_coordinates + robot_position;

    // final orientation is the parameter that can control the robot to rotate in the
    // direction of angular velocity from current robot orientation, clamp the angular
    // velocity between [-pi/2,pi/2]
    Angle final_orientation =
        robot_orientation +
        Angle::ofRadians(direct_velocity_primitive.getAngularVelocity())
            .mod(Angle::half());

    motion_controller_command = MotionController::PositionCommand(
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
    Point kick_origin    = kick_primitive.getKickOrigin();
    Angle kick_direction = kick_primitive.getKickDirection();

    double final_speed_at_destination = 0.0;

    Point point_behind = kick_origin + Vector(Point::createFromAngle(kick_direction).x(),
                                              Point::createFromAngle(kick_direction).y());

    Point closest_point_to_line =
        closestPointOnLine(robot.position(), kick_origin, point_behind);

    // If current robot position is in line with the shot (i.e. less than two robot radius
    // within the line in the direction of the shot), can go for the shot
    if (offsetToLine(kick_origin, point_behind, robot.position()) <=
        2 * ROBOT_MAX_RADIUS_METERS)
    {
        motion_controller_command = MotionController::PositionCommand(
            kick_origin, kick_direction, final_speed_at_destination,
            kick_primitive.getKickSpeed(), false, false);
    }
    else
    {
        // If robot is not in line, move to the closest point on the line from the current
        // position
        motion_controller_command = MotionController::PositionCommand(
            closest_point_to_line, kick_direction, 0.0, kick_primitive.getKickSpeed(),
            false, false);
        ;
    }
}

void GrsimCommandPrimitiveVisitor::visit(const MovePrimitive &move_primitive)
{
    motion_controller_command = MotionController::PositionCommand(
        move_primitive.getDestination(), move_primitive.getFinalAngle(),
        move_primitive.getFinalSpeed(), 0.0, false, false);
}

void GrsimCommandPrimitiveVisitor::visit(const MoveSpinPrimitive &move_spin_primitive)
{
    Angle targetAngle = robot.orientation();

    (move_spin_primitive.getAngularVelocity() > AngularVelocity::zero()
         ? targetAngle += Angle::ofDegrees(45)
         : targetAngle -= Angle::ofDegrees(45));

    motion_controller_command = MotionController::PositionCommand(
        move_spin_primitive.getDestination(), targetAngle, 0.0, 0.0, false, false);
}

void GrsimCommandPrimitiveVisitor::visit(const PivotPrimitive &pivot_primitive)
{
    // Compute final position
    Point final_robot_position(
        pivot_primitive.getPivotPoint().x() +
            (pivot_primitive.getPivotRadius() * pivot_primitive.getFinalAngle().cos()),
        pivot_primitive.getPivotPoint().y() +
            (pivot_primitive.getPivotRadius() * pivot_primitive.getFinalAngle().sin()));

    Vector robot_to_pivot_point = robot.position() - pivot_primitive.getPivotPoint();

    // find the general direction to travel
    Vector general_direction_to_destination = final_robot_position - robot.position();

    // there are two directions to rotate CW and CCW to consider
    Vector unit_robot_to_pivot_point = robot_to_pivot_point.norm();
    Vector tangential_dir_1(unit_robot_to_pivot_point.y(),
                            -unit_robot_to_pivot_point.x());
    Vector tangential_dir_2(-unit_robot_to_pivot_point.y(),
                            unit_robot_to_pivot_point.x());

    // based on how much of the general direction projects onto the two tangential
    // direction vector, the direction is selected. The one with the higher magnitude
    // is selected, as it will be the shortest path to rotate
    float weight_of_direction_1 = general_direction_to_destination.dot(tangential_dir_1);
    float weight_of_direction_2 = general_direction_to_destination.dot(tangential_dir_2);

    // get tangential and radial directions
    Vector tangential_dir = (weight_of_direction_1 >= weight_of_direction_2)
                                ? tangential_dir_1
                                : tangential_dir_2;

    // compute correction based on current radius betwen pivot point and robot to the
    // given pivot radius
    float correction = pivot_primitive.getPivotRadius() - robot_to_pivot_point.len();

    // gets the magnitude of a straight line from the current position and the final
    // position this value is used to scale the speed
    double linear_displacement_to_final_position =
        (final_robot_position - robot.position()).len();

    Vector final_vector = (correction * unit_robot_to_pivot_point) +
                          (linear_displacement_to_final_position * tangential_dir);

    motion_controller_command = MotionController::VelocityCommand(
        0.0, 0.0, false, final_vector,
        robot.orientation() - unit_robot_to_pivot_point.orientation());
}

void GrsimCommandPrimitiveVisitor::visit(const DribblePrimitive &dribble_primitive)
{
    motion_controller_command = MotionController::PositionCommand(
        dribble_primitive.getDestination(), dribble_primitive.getFinalAngle(),
        dribble_primitive.getFinalSpeed(), 0.0, false, true);
}

void GrsimCommandPrimitiveVisitor::visit(const StopPrimitive &stop_primitive)
{
    // intentionally leaving out the option to coast until later
    motion_controller_command = MotionController::PositionCommand(
        robot.position(), robot.orientation(), 0, 0.0, false, false);
}

MotionControllerCommand GrsimCommandPrimitiveVisitor::getMotionControllerCommand()
{
    return motion_controller_command;
}

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

    motion_controller_command = MotionController::MotionControllerCommand(
        finalDest, robotDirection, catch_primitive.getVelocity(), 0.0, false,
        catch_primitive.getDribblerSpeed() > 0);
}

void GrsimCommandPrimitiveVisitor::visit(const ChipPrimitive &chip_primitive)
{
    Point chip_origin    = chip_primitive.getChipOrigin();
    Angle chip_direction = chip_primitive.getChipDirection();

    double final_speed_at_destination = 0.0;
    Point perp_line_to_shot = chip_origin + Point::createFromAngle(chip_direction).perp();
    double dest_behind_x    = -cos(chip_direction.toDegrees());
    double dest_behind_y    = -sin(chip_direction.toDegrees());
    Point dest_behind       = chip_origin + Vector(dest_behind_x, dest_behind_y);

    // If current distance between robot and line perpendicular to shot is
    // less than distance between destination behind shot and line perpendicular to shot
    // while robot is in same direction as shot, can go for the shot
    if (!(offsetToLine(chip_origin, dest_behind, robot.position()) <=
              ROBOT_MAX_RADIUS_METERS &&
          offsetToLine(chip_origin, perp_line_to_shot, robot.position()) <=
              offsetToLine(chip_origin, perp_line_to_shot, dest_behind)))
    {
        motion_controller_command = MotionController::MotionControllerCommand(
            dest_behind, chip_direction, 0.0, chip_primitive.getChipDistance(), true,
            false);
    }
    else
    {
        motion_controller_command = MotionController::MotionControllerCommand(
            chip_origin, chip_direction, final_speed_at_destination,
            chip_primitive.getChipDistance(), true, false);
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
    Point kick_origin    = kick_primitive.getKickOrigin();
    Angle kick_direction = kick_primitive.getKickDirection();

    double final_speed_at_destination = 0.0;
    // double dest_behind_x    = -cos(kick_direction.toDegrees());
    // double dest_behind_y    = -sin(kick_direction.toDegrees());
    double dest_behind_x = -Point::createFromAngle(kick_direction).x();
    double dest_behind_y = -Point::createFromAngle(kick_direction).y();

    Point dest_behind       = kick_origin + Vector(dest_behind_x, dest_behind_y);
    // If current distance between robot and line perpendicular to shot is
    // less than distance between destination behind shot and line perpendicular to shot
    // while robot is in same direction as shot, can go for the shot
    if (!(offsetToLine(kick_origin, dest_behind, robot.position()) <= ROBOT_MAX_RADIUS_METERS))
    {
        motion_controller_command = MotionController::MotionControllerCommand(
            dest_behind, kick_direction, 0.0, kick_primitive.getKickSpeed(), false,
            false);
    }
    else
    {
        motion_controller_command = MotionController::MotionControllerCommand(
            kick_origin, kick_direction, final_speed_at_destination,
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
    Angle targetAngle = robot.orientation();

    (move_spin_primitive.getAngularVelocity() > AngularVelocity::zero()
         ? targetAngle += Angle::ofDegrees(45)
         : targetAngle -= Angle::ofDegrees(45));

    motion_controller_command = MotionController::MotionControllerCommand(
        move_spin_primitive.getDestination(), targetAngle, 0.0, 0.0, false, false);
}

void GrsimCommandPrimitiveVisitor::visit(const PivotPrimitive &pivot_primitive)
{
    // TODO: https://github.com/UBC-Thunderbots/Software/issues/94
}

void GrsimCommandPrimitiveVisitor::visit(const DribblePrimitive &dribble_primitive)
{
    // TODO: https://github.com/UBC-Thunderbots/Software/issues/107
}

void GrsimCommandPrimitiveVisitor::visit(const StopPrimitive &stop_primitive)
{
    // intentionally leaving out the option to coast until later
    motion_controller_command = MotionController::MotionControllerCommand(
        robot.position(), robot.orientation(), 0, 0.0, false, false);
}

MotionController::MotionControllerCommand
GrsimCommandPrimitiveVisitor::getMotionControllerCommand()
{
    return motion_controller_command;
}

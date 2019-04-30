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
    // compute final position
    Point final_robot_position(
        pivot_primitive.getPivotPoint().x() +
            (pivot_primitive.getPivotRadius() * pivot_primitive.getFinalAngle().cos()),
        pivot_primitive.getPivotPoint().y() +
            (pivot_primitive.getPivotRadius() * pivot_primitive.getFinalAngle().sin()));

    // get current vector from pivot point to robot position
    Vector unit_pivot_point_to_robot_pos =
        (pivot_primitive.getPivotPoint() - robot.position()).norm();

    // get a vector in the tangential direction
    Vector tangential_dir_1(unit_pivot_point_to_robot_pos.y(),
                            -unit_pivot_point_to_robot_pos.x());
    Vector tangential_dir_2(-unit_pivot_point_to_robot_pos.y(),
                            unit_pivot_point_to_robot_pos.x());

    // get collinear point on orbit, between the robot and the pivot point, used
    // to maintain orbit
    Point collinear_point_on_orbit =
        pivot_primitive.getPivotPoint() +
        pivot_primitive.getPivotRadius() * -unit_pivot_point_to_robot_pos;

    // the robot can take go to two tangential points on the nex call
    // based on which one is closer to the final destination
    Point robot_next_position_1 = collinear_point_on_orbit + tangential_dir_1;
    Point robot_next_position_2 = collinear_point_on_orbit + tangential_dir_2;

    // comparing the magnitude of vector from the two possible next positions to the final
    // position, will tell the robot which way to move. The shorter magnitude is the
    // shorter path on orbit. since this is resolved every time this function is called,
    // if pivot overshoots, it will rotate the other way
    Vector tangential_vector =
        (robot_next_position_1 - final_robot_position).len() <
                (robot_next_position_2 - final_robot_position).len()
            ? tangential_dir_1
            : tangential_dir_2;

    // get displacement to final robot position, if the robot were to move there linearly.
    // This value is used to scale the tangential vector added to the collinear point to
    // allow for rotation.
    Vector linear_displacement_to_final_robot_position =
        final_robot_position - robot.position();

    // always move to collinear point on orbit, plus a portion in the tangential direction
    // based on how much linear displacement is left from the current and final position
    // NOTE: Scaling the displacement by a half, ensures that the robot slows down more
    // aggressively
    motion_controller_command = MotionController::PositionCommand(
        collinear_point_on_orbit +
            tangential_vector * 0.5 * linear_displacement_to_final_robot_position.len(),
        unit_pivot_point_to_robot_pos.orientation(), 0, 0.0, false, false);
}

void GrsimCommandPrimitiveVisitor::visit(const DribblePrimitive &dribble_primitive)
{
    motion_controller_command = MotionController::PositionCommand(
        dribble_primitive.getDestination(), dribble_primitive.getFinalAngle(), 0.0, 0.0,
        false, true);
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

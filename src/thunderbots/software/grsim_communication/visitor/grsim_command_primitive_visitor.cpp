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
        finalDest = ball.estimatePositionAtFutureTime(std::chrono::milliseconds(
            (int)fabs(distanceToBall / robot.velocity().len()) * 1000));
        finalDest = Point(finalDest.x(), finalDest.y());
    }
    else
    {
        // If Robot is not moving, estimate position based on a standard velocity of 1
        finalDest = ball.estimatePositionAtFutureTime(
            std::chrono::milliseconds((int)fabs(distanceToBall / 1) * 1000));
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

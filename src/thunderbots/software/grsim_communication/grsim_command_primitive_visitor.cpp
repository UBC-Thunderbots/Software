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
    // TODO: https://github.com/UBC-Thunderbots/Software/issues/93
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
    // TODO: https://github.com/UBC-Thunderbots/Software/issues/93
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
        pivot_primitive.getPivotPoint() +
            pivot_primitive.getPivotRadius() * pivot_primitive.getFinalAngle().cos(),
        pivot_primitive.getPivotPoint() +
            pivot_primitive.getPivotRadius() * pivot_primitive.getFinalAngle().sin());

    Vector pivot_point_to_robot = pivot_primitive.getPivotPoint() - robot.position();
    Vector unit_pivot_point_to_robot = (pivot_primitive.getPivotPoint() - robot.position())/pivot_point_to_robot.len();

    // find the general direction to travel, project those onto the two possible
    // directions and see which one is better
    Vector general_direction_to_destination = final_robot_position - robot.position();

    // there are two directions to rotate CW and CCW, the one with the 
    Vector tangential_dir_1(unit_pivot_point_to_robot.y(), -unit_pivot_point_to_robot.x());
    Vector tangential_dir_2(-unit_pivot_point_to_robot.y(), unit_pivot_point_to_robot.x());

    // based on how much of the general direction projects onto the two tangential 
    // direction vector, the direction is selected. The one with the lower mangnitude 
    // is selected, as it will be the shortest path to rotate
    float weight_of_direction_1 = general_dir.dot(tangential_dir_1);
    float weight_of_direction_2 = general_dir.dot(tangential_dir_2);

    // get tangential and radial directions
    Vector tangential_dir = (weight_of_direction_1 >= weight_of_direction_1) ? tangential_dir_1 : tangential_dir_2;
    Vector radial_dir = unit_pivot_point_to_robot;
    
    // compute correction based on current radius betwen pivot point and robot to the given pivot radius
    float correction = pivot_point_to_robot.len() - pivot_primitive.getPivotRadius();

    // gets the magnitude of a straight line from the current position and the final position 
    // this value is used to scale the speed
    double linear_displacement_to_final_position = (final_robot_position - robot.position()).len();
    float disp_to_final_dest = compute_magnitude(end_goal_vect);

    // TODO bring it to zero if its close enough (disp_to_final_dest)

    // figure out all velocities in prioritized directions
    float current_rot_vel = dot_product(tangential_dir, vel, 2);
    float current_cor_vel = dot_product(radial_dir, vel, 2);

    float mag_accel_orbital = compute_acceleration(
        &rotation_profile, disp_to_final_dest, current_rot_vel, STOPPED, MAX_A, MAX_V);
    float mag_accel_correction = compute_acceleration(
        &correction_profile, correction, current_cor_vel, STOPPED, MAX_A, MAX_V);

    PositionCommand(Angle final_orientation,
                    double kick_or_chip_power,
                    bool chip_instead_of_kick,
                    bool dribbler_on,
                    Vector requested_linear_velocity,
                    AngularVelocity requested_angular_velocity)

    // add the 3 directions together
    float accel[3] = {0};

    accel[0] = mag_accel_correction * dot_product(radial_dir, local_x_norm_vec, 2);
    accel[1] = mag_accel_correction * dot_product(radial_dir, local_y_norm_vec, 2);

    if (WITHIN_THRESH(correction))
    {
        accel[0] += mag_accel_orbital * dot_product(tangential_dir, local_x_norm_vec, 2);
        accel[1] += mag_accel_orbital * dot_product(tangential_dir, local_y_norm_vec, 2);
    }

    // destination
    float angle =
        min_angle_delta(current_bot_state.angle, atan2f(rel_dest[1], rel_dest[0]));
}

void GrsimCommandPrimitiveVisitor::visit(const DribblePrimitive &dribble_primitive)
{
    // TODO: https://github.com/UBC-Thunderbots/Software/issues/107
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

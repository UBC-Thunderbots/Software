/**
 * .cpp file for the grSim motion controller.
 *
 * In the current state it is a bang-bang controller.
 *
 * It assumed the robot max acceleration is constant.
 *
 * Uses constant acceleration kinematics equations to
 * calculate changes in speed.
 *
 * See https://en.wikipedia.org/wiki/Bang%E2%80%93bang_control for more info
 */

#include "robot_communication/grsim/motion_controller.h"

#include <chrono>
#include <iostream>
#include <utility>

#include "ai/world/robot.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "shared/constants.h"


MotionController::Velocity MotionController::bangBangVelocityController(
    const Robot robot, const Point dest, const double desired_final_speed,
    const Angle desired_final_orientation, const double delta_time)
{
    MotionController::Velocity robot_velocities;

    // if the change is time is somehow negative or zero, just return the current robot
    // velocity
    // TODO: Implement exception handling for negative time case
    // See issue #123
    if (delta_time <= 0)
    {
        robot_velocities.linear_velocity  = robot.velocity();
        robot_velocities.angular_velocity = robot.angularVelocity();
    }
    else
    {
        robot_velocities.linear_velocity = MotionController::determineLinearVelocity(
            robot, dest, desired_final_speed, delta_time);
        robot_velocities.angular_velocity = MotionController::determineAngularVelocity(
            robot, desired_final_orientation, delta_time);
    }

    return robot_velocities;
}

AngularVelocity MotionController::determineAngularVelocity(
    const Robot robot, const Angle desired_final_orientation, const double delta_time)
{
    // rotation used for constant angular acceleration calculations
    const Angle delta_angle = (desired_final_orientation - robot.orientation());

    AngularVelocity robot_angular_velocity = robot.angularVelocity();

    // integer that will be -1 or 1 giving the rotation direction to reach the destination
    const int rotation_direction_modifier =
        delta_angle.toRadians() / delta_angle.abs().toRadians();

    // calculate the change in angular speed as: max_acceleration * delta_time
    AngularVelocity delta_angular_speed = AngularVelocity::ofRadians(
        rotation_direction_modifier * ROBOT_MAX_ANG_ACCELERATION * delta_time);

    // check if the robot is nearly in it's final orientation with close to no angular
    // velocity
    if ((delta_angle).abs().toRadians() <= POSITION_STOP_TOLERANCE &&
        robot.angularVelocity().toRadians() <= VELOCITY_STOP_TOLERANCE)
    {
        return AngularVelocity::ofRadians(0);
    }

    // check to see if the robot can deaccelerate angularly in time to reach it's target
    // orientation with zero angular velocity based on equation Wf = sqrt( Wi^2 -
    // 2*alpha*delta_angle ) where we are checking Wi^2 <= 2*alpha*delta_angle
    bool can_stop_rotate_in_time =
        (pow(robot.angularVelocity().toRadians(), 2) <=
         fabs(2 * ROBOT_MAX_ANG_ACCELERATION * delta_angle.toRadians()));

    // check robot rotation state for correct delta_angular_speed sign
    if (!can_stop_rotate_in_time)
    {
        delta_angular_speed *= 1;
    }

    // calculate the new angular velocity
    robot_angular_velocity = robot.angularVelocity() + delta_angular_speed;

    // check if the calculated angular speed is higher than the allowed maximum for the
    // robot
    if (robot_angular_velocity.toRadians() > ROBOT_MAX_ANG_SPEED)
    {
        robot_angular_velocity = AngularVelocity::ofRadians(ROBOT_MAX_ANG_SPEED);
    }
    else if (robot_angular_velocity.toRadians() < -ROBOT_MAX_ANG_SPEED)
    {
        robot_angular_velocity = AngularVelocity::ofRadians(-ROBOT_MAX_ANG_SPEED);
    }

    return robot_angular_velocity;
}

Vector MotionController::determineLinearVelocity(const Robot robot, const Point dest,
                                                 const double desired_final_speed,
                                                 const double delta_time)
{
    // vector to hold the XY velocities of the robot
    Vector robot_linear_velocities;

    // destination distance used for constant linear acceleration speed calculations
    const double distance_to_dest = (robot.position() - dest).len();

    const double max_delta_speed = ROBOT_MAX_ACCELERATION * delta_time;

    // contain the change in linear speed of the robot
    double delta_speed_x = 0;
    double delta_speed_y = 0;


    // if the robot is required to stop at a destination check to see if it is "close
    // enough" and return zero speed to avoid jittering
    if (distance_to_dest <= POSITION_STOP_TOLERANCE &&
        robot.velocity().len() <= VELOCITY_STOP_TOLERANCE && desired_final_speed == 0)
    {
        return Vector(0, 0);
    }

    // calculate the angle between the robot and it's destination
    const Angle angle_to_destination = (dest - robot.position()).norm().orientation();

    // calculate the angle between the robot's velocity and the angle to it's destination.
    // velocity_error_angle will be used to calculate the error in the robots velocity
    // to it's ideal velocity pointing directly at the destination
    const Angle velocity_error_angle =
        (angle_to_destination - robot.velocity().orientation());


    // calculate the robot's velocity projection along the axis towards it's destination
    // and determine if it can stop in time
    bool can_stop_in_time =
        ((double)pow(robot.velocity().project(dest).len(), 2) -
         2 * ROBOT_MAX_ACCELERATION * distance_to_dest) <= desired_final_speed;

    // Case 1: Robot is moving slower than speed tolerance (can't trust velocity angle due
    // to noise)
    if (robot.velocity().len() < VELOCITY_STOP_TOLERANCE ||
        (fabs(velocity_error_angle.toDegrees()) <= 1 && can_stop_in_time))
    {
        delta_speed_x = max_delta_speed * angle_to_destination.cos();
        delta_speed_y = max_delta_speed * angle_to_destination.sin();
    }

    // Case 2: Robot velocity is opposite to the direction of the destination
    else if (can_stop_in_time && (fabs(velocity_error_angle.toDegrees()) > 179 &&
                                  (fabs(velocity_error_angle.toDegrees()) <= 181)))
    {
        delta_speed_x = max_delta_speed *
                        (velocity_error_angle + robot.velocity().orientation()).cos();
        delta_speed_y = max_delta_speed *
                        (velocity_error_angle + robot.velocity().orientation()).sin();
    }

    // Case 3: Robot can stop in time (Accelerate and correct velocity angle)
    else if (can_stop_in_time)
    {
        delta_speed_x =
            max_delta_speed * (velocity_error_angle + angle_to_destination).cos();
        delta_speed_y =
            max_delta_speed * (velocity_error_angle + angle_to_destination).sin();
    }

    // Case 4: Robot can't stop in time (accelerate opposite to robot velocity)
    else
    {
        delta_speed_x = max_delta_speed *
                        (robot.velocity().orientation() + Angle::ofDegrees(180)).cos();
        delta_speed_y = max_delta_speed *
                        (robot.velocity().orientation() + Angle::ofDegrees(180)).sin();
    }

    // calculate new X/Y velocities based on the
    // current robot speeds and calculated delta speeds
    robot_linear_velocities = Vector(robot.velocity().x() + delta_speed_x,
                                     robot.velocity().y() + delta_speed_y);

    if (robot_linear_velocities.len() >= ROBOT_MAX_SPEED)
    {
        robot_linear_velocities = robot_linear_velocities.norm(ROBOT_MAX_SPEED);
    }

    return robot_linear_velocities;
}

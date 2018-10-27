#include "backend_output/grsim/motion_controller.h"

#include <chrono>
#include <iostream>
#include <utility>

#include "ai/world/robot.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "shared/constants.h"

/**
 *.cpp file for the grSim motion controller.
 *
 *In the current state it is a bang-bang controller.
 *
 *It assumed the robot max acceleration is constant.
 *
 *Uses constant acceleration kinematics equations to
 *calculate changes in speed.
 *
 *See https://en.wikipedia.org/wiki/Bang%E2%80%93bang_control for more info
 */
MotionController::Velocity MotionController::bangBangVelocityController(
    const Robot robot, const Point dest, const double desired_final_speed,
    const Angle desired_final_orientation, const double delta_time)
{
    MotionController::Velocity robot_velocities;

    // if the change is time is somehow negative or zero, just return the current robot
    // velocity
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
    // boolean value if the robot can reach it's destination at target speed based on
    // angular MAX acceleration
    bool can_stop_rotate_in_time;
    bool rotating_towards_dest;

    // check the direction of rotation of the robot
    bool angular_velocity_is_positive = robot.angularVelocity().toRadians() > 0;
    AngularVelocity robot_angular_velocity;

    // rotation used for constant angular acceleration calculations
    const Angle angle_to_dest = (desired_final_orientation - robot.orientation());
    // integer that will be -1 or 1 giving the rotation direction to reach the destination
    const int direction_to_dest = angle_to_dest.toRadians()/angle_to_dest.abs().toRadians();

    AngularVelocity delta_angular_speed;

    // check if the robot is nearly in it's final orientation with close to no angular
    // velocity
    if ((robot.orientation() - desired_final_orientation).abs().toRadians() <= 0.01 &&
        robot.angularVelocity().toRadians() <= 0.015)
    {
        return AngularVelocity::ofRadians(0);
    }

    // check if the robot is rotating in the same direction as it's target angle
    if ((desired_final_orientation - robot.orientation()).abs().toRadians() > 0 &&
        robot.angularVelocity().abs().toRadians() > 0)
    {
        rotating_towards_dest = true;
    }
    else
    {
        rotating_towards_dest = false;
    }

    // check to see if the robot can deaccelerate angularly in time to reach it's target
    // orientation with zero angular velocity based on equation Wf = sqrt( Wi^2 -
    // 2*alpha*angle_to_dest ) where we are checking Wi^2 <= 2*alpha*angle_to_dest
    if (pow(robot.angularVelocity().toRadians(), 2) <=
        fabs(2 * ROBOT_MAX_ANG_ACCELERATION * angle_to_dest.toRadians()))
    {
        can_stop_rotate_in_time = true;
    }
    else
    {
        can_stop_rotate_in_time = false;
    }

    // check robot rotation state for correct delta_angular_speed sign
    if (rotating_towards_dest && can_stop_rotate_in_time)
    {
//        if (angular_velocity_is_positive)
//        {
//            delta_angular_speed =
//                AngularVelocity::ofRadians(ROBOT_MAX_ANG_ACCELERATION * delta_time);
//        }
//        else
//        {
//            delta_angular_speed =
//                AngularVelocity::ofRadians(-ROBOT_MAX_ANG_ACCELERATION * delta_time);
//        }
        delta_angular_speed = AngularVelocity::ofRadians(direction_to_dest*ROBOT_MAX_ANG_SPEED*delta_time);
    }
    // if the robot is rotating in the correct direction but can't stop in time, slow down
    else if (rotating_towards_dest && !can_stop_rotate_in_time)
    {
//        if (angular_velocity_is_positive)
//        {
//            delta_angular_speed =
//                AngularVelocity::ofRadians(-ROBOT_MAX_ANG_ACCELERATION * delta_time);
//        }
//        else
//        {
//            delta_angular_speed =
//                AngularVelocity::ofRadians(ROBOT_MAX_ANG_ACCELERATION * delta_time);
//        }
        delta_angular_speed = AngularVelocity::ofRadians(-direction_to_dest*ROBOT_MAX_ANG_SPEED*delta_time);
    }
        // special case that the robot has no angular velocity to determine the rotation
        // direction

    else if (robot.angularVelocity().toRadians() == 0)
    {
        if ((desired_final_orientation - robot.orientation()).toRadians() > 0)
        {
            delta_angular_speed =
                    AngularVelocity::ofRadians(ROBOT_MAX_ANG_ACCELERATION * delta_time);
        }
        else
        {
            delta_angular_speed =
                    AngularVelocity::ofRadians(-ROBOT_MAX_ANG_ACCELERATION * delta_time);
        }
    }
    // the last case is that the robot is not roating towards it's final angle, which
    // means it should accelerate the other direction
    else
    {
//        if (angular_velocity_is_positive)
//        {
//            delta_angular_speed =
//                AngularVelocity::ofRadians(-ROBOT_MAX_ANG_ACCELERATION * delta_time);
//        }
//        else
//        {
//            delta_angular_speed =
//                AngularVelocity::ofRadians(ROBOT_MAX_ANG_ACCELERATION * delta_time);
//        }
        delta_angular_speed = AngularVelocity::ofRadians(direction_to_dest*ROBOT_MAX_ANG_SPEED*delta_time);
    }

    // calculate the new angular velocity
    robot_angular_velocity = robot.angularVelocity() + delta_angular_speed;

    // check if the calculated angular speed is higher than the allowed maximum for the
    // robot
    if ((robot_angular_velocity.abs() > AngularVelocity::ofRadians(ROBOT_MAX_ANG_SPEED)))
    {
        if (robot_angular_velocity < AngularVelocity::ofRadians(0))
        {
            robot_angular_velocity = AngularVelocity::ofRadians(-1 * ROBOT_MAX_ANG_SPEED);
        }
        else
        {
            robot_angular_velocity = AngularVelocity::ofRadians(ROBOT_MAX_ANG_SPEED);
        }
    }

    return robot_angular_velocity;
}

Vector MotionController::determineLinearVelocity(const Robot robot, const Point dest,
                                                 const double desired_final_speed,
                                                 const double delta_time)
{
    // vector to hold the XY velocities of the robot
    Vector robot_linear_velocities;

    bool moving_towards_dest_x;
    bool moving_towards_dest_y;
    bool can_stop_in_time;

    // destination distance used for constant linear acceleration speed calculations
    const double distance_to_dest = (robot.position() - dest).len();

    // calculates robot angle based on unit vector that points from the robot location to
    // the destination (used to calculate the X/Y velocity magnitudes
    const Angle direction_angle = (dest - robot.position()).norm().orientation();

    double delta_speed_x, delta_speed_y;

    if ((robot.position() - dest).len() <= 0.01 && robot.velocity().len() <= 0.015)
    {
        return Vector(0, 0);
    }

    // if robot is moving towards destination in x
    if ((dest.x() - robot.position().x() > 0) && (robot.velocity().x() > 0) ||
        (dest.x() - robot.position().x() < 0) && (robot.velocity().x() < 0))
    {
        moving_towards_dest_x = true;
    }
    else
    {
        moving_towards_dest_x = false;
    }

    // if robot is moving towards destination in y
    if ((dest.x() - robot.position().x() > 0) && (robot.velocity().x() > 0) ||
        (dest.x() - robot.position().x() < 0) && (robot.velocity().x() < 0))
    {
        moving_towards_dest_y = true;
    }
    else
    {
        moving_towards_dest_y = false;
    }

    // calculate if the robot can stop in time to achieve the target speed at the target
    // destination based on acceleration Vf = sqrt( Vi^2 - 2*a*d) if the robot can stop in
    // time ( Vi^2 < 2*a*d
    if ((pow(robot.velocity().len(), 2) - 2 * ROBOT_MAX_ACCELERATION * distance_to_dest) <
        desired_final_speed)
    {
        can_stop_in_time = true;
    }
    // check for directions
    if (moving_towards_dest_x && moving_towards_dest_y)
    {
        if (can_stop_in_time)
        {
            delta_speed_x = (ROBOT_MAX_ACCELERATION * delta_time) * direction_angle.cos();
            delta_speed_y = (ROBOT_MAX_ACCELERATION * delta_time) * direction_angle.sin();
        }
        else
        {
            delta_speed_x =
                -(ROBOT_MAX_ACCELERATION * delta_time) * direction_angle.cos();
            delta_speed_y =
                -(ROBOT_MAX_ACCELERATION * delta_time) * direction_angle.sin();
        }
    }
    else if (moving_towards_dest_x && !moving_towards_dest_y)
    {
        if (can_stop_in_time)
        {
            delta_speed_x = (ROBOT_MAX_ACCELERATION * delta_time) * direction_angle.cos();
            delta_speed_y =
                -(ROBOT_MAX_ACCELERATION * delta_time) * direction_angle.sin();
        }
        else
        {
            delta_speed_x =
                -(ROBOT_MAX_ACCELERATION * delta_time) * direction_angle.cos();
            delta_speed_y = (ROBOT_MAX_ACCELERATION * delta_time) * direction_angle.sin();
        }
    }
    else if (!moving_towards_dest_x && moving_towards_dest_y)
    {
        if (can_stop_in_time)
        {
            delta_speed_x =
                -(ROBOT_MAX_ACCELERATION * delta_time) * direction_angle.cos();
            delta_speed_y = (ROBOT_MAX_ACCELERATION * delta_time) * direction_angle.sin();
        }
        else
        {
            delta_speed_x = (ROBOT_MAX_ACCELERATION * delta_time) * direction_angle.cos();
            delta_speed_y =
                -(ROBOT_MAX_ACCELERATION * delta_time) * direction_angle.sin();
        }
    }
    else
    {
        // if the robot is moving in the opposite direction of the destination, then slow
        // down
        delta_speed_x = (ROBOT_MAX_ACCELERATION * delta_time) * direction_angle.cos();
        delta_speed_y = (ROBOT_MAX_ACCELERATION * delta_time) * direction_angle.sin();
    }

    robot_linear_velocities =
        Vector(robot.velocity().x() + delta_speed_x,
               robot.velocity().y() +
                   delta_speed_y);  // calculate new X/Y/ang velocities based on the
    // current robot speeds and delta speeds

    if (robot_linear_velocities.len() >= ROBOT_MAX_SPEED)
    {
        robot_linear_velocities = robot_linear_velocities.norm(ROBOT_MAX_SPEED);
    }

    return robot_linear_velocities;
}

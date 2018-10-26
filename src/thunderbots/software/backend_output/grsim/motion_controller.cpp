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

    robot_velocities.linear_velocity = MotionController::determineLinearVelocity(
        robot, dest, desired_final_speed, delta_time);
    robot_velocities.angular_velocity = MotionController::determineAngularVelocity(
        robot, desired_final_orientation, delta_time);

    return robot_velocities;
}

AngularVelocity MotionController::determineAngularVelocity(
    const Robot robot, const Angle desired_final_orientation, const double delta_time)
{
    // boolean value if the robot can reach it's destination at target speed based on
    // angular MAX acceleration
    bool can_stop_rotate_in_time;

    AngularVelocity robot_angular_velocity;

    // rotation used for constant angular acceleration calculations
    const Angle angle_to_dest = (robot.orientation() - desired_final_orientation);
    AngularVelocity delta_angular_speed;
    AngularVelocity expected_final_ang_speed;

    // check negative sqrt case for angular speed
    if (pow(robot.angularVelocity().toRadians(), 2) <=
        fabs(2 * ROBOT_MAX_ANG_ACCELERATION * angle_to_dest.toRadians()))
    {
        expected_final_ang_speed = AngularVelocity::ofRadians(
            -1 * sqrt(fabs(2 * ROBOT_MAX_ANG_ACCELERATION * angle_to_dest.toRadians()) -
                      pow(robot.angularVelocity().toRadians(), 2)));
        can_stop_rotate_in_time = true;
    }
    else
    {
        expected_final_ang_speed = AngularVelocity::ofRadians(
            sqrt(pow(robot.angularVelocity().toRadians(), 2) -
                 2 * ROBOT_MAX_ANG_ACCELERATION * angle_to_dest.toRadians()));
        can_stop_rotate_in_time =
            expected_final_ang_speed <= AngularVelocity::ofRadians(0.0);
    }

    // if the robot can stop rotating in time
    if (can_stop_rotate_in_time)
    {
        // if the final expected angle is less than the desired angle then angularly
        // accelerate
        if (expected_final_ang_speed < AngularVelocity::ofRadians(0.0))
        {
            // angularly accelerate
            delta_angular_speed =
                AngularVelocity::ofRadians(ROBOT_MAX_ANG_ACCELERATION * delta_time);
        }

        // if not then maintain angular velocity
        else
        {
            delta_angular_speed = AngularVelocity::ofRadians(0.0);
        }
    }

    // if the robot can't stop rotating in time then angular decelerate
    else
    {
        delta_angular_speed =
            AngularVelocity::ofRadians(-(ROBOT_MAX_ANG_ACCELERATION * delta_time));
    }

    robot_angular_velocity =
        robot.angularVelocity() +
        delta_angular_speed;  // calculate new angular velocity based on the current robot
    // ang. velocity and delta ang. velocity

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

    // boolean value if the robot can reach it's destination at target speed based on
    // linear MAX acceleration
    bool can_stop_in_time;

    // destination distance used for constant linear acceleration speed calculations
    const double distance_to_dest = (robot.position() - dest).len();

    // calculates robot angle based on unit vector that points from the robot location to
    // the destination (used to calculate the X/Y velocity magnitudes
    const Angle direction_angle = (dest - robot.position()).norm().orientation();
    double expected_final_speed;

    double delta_speed_x, delta_speed_y;

    // calculate the expected speed at the destination based on current speed and
    // acceleration Vf = sqrt( Vi^2 - 2*a*d)

    // check for negative sqrt case
    if (pow(robot.velocity().len(), 2) <= 2 * ROBOT_MAX_ACCELERATION * distance_to_dest)
    {
        // if the sqrt is negative, the final speed will be negative (opposite direction
        // of current speed)
        expected_final_speed = -1 * sqrt(2 * ROBOT_MAX_ACCELERATION * distance_to_dest -
                                         pow(robot.velocity().len(), 2));
        can_stop_in_time     = true;
    }
    else
    {
        // calculate the expected final speed assuming max decceleration
        expected_final_speed = sqrt(pow(robot.velocity().len(), 2) -
                                    2 * ROBOT_MAX_ACCELERATION * distance_to_dest);

        // the robot can stop in time if it's desired final speed is higher than the speed
        // if the robot maximum decelerates from the current state
        can_stop_in_time = expected_final_speed <= desired_final_speed;
    }

    if (can_stop_in_time)
    {
        // if the robot can stop in time and is going slower than the desired final speed
        // then accelerate
        if (expected_final_speed < desired_final_speed)
        {
            delta_speed_x = (ROBOT_MAX_ACCELERATION * delta_time) * direction_angle.cos();
            delta_speed_y = (ROBOT_MAX_ACCELERATION * delta_time) * direction_angle.sin();
        }

        // if not then maintain speed
        else
        {
            // maintain speed
            delta_speed_x = 0.0;
            delta_speed_y = 0.0;
        }
    }

    // if the robot can't stop in time then decelerate
    else
    {
        delta_speed_x = -(ROBOT_MAX_ACCELERATION * delta_time) * direction_angle.cos();
        delta_speed_y = -(ROBOT_MAX_ACCELERATION * delta_time) * direction_angle.sin();
    }

    robot_linear_velocities =
        Vector(robot.velocity().x() + delta_speed_x,
               robot.velocity().y() +
                   delta_speed_y);  // calculate new X/Y/ang velocities based on the
    // current robot speeds and delta speeds

    if (robot_linear_velocities.len() > ROBOT_MAX_SPEED)
    {
        robot_linear_velocities = robot_linear_velocities.norm(ROBOT_MAX_SPEED);
    }

    return robot_linear_velocities;
}

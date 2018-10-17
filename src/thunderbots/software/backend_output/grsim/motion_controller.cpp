//
// Created by evan on 18/08/18.
//

#include "motion_controller.h"

#include <chrono>
#include <ctime>
#include <iostream>
#include <utility>

#include "ai/world/robot.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "shared/constants.h"


// .cpp file for the grSim motion controller.
//
// In the current state it is a bang-bang controller.
//
// It assumed the robot max acceleration is constant.
//
// Uses constant acceleration kinematics equations to
// calculate changes in speed.

std::pair<Vector, Angle> MotionController::grSimBangBang(
    Robot robot, Point dest, const double desired_final_speed,
    const Angle desired_final_orientation, double delta_time)
{
    Vector robot_linear_velocities;  // vector to hold the XY velocities of the robot
    bool b_can_stop_in_time;  // boolean value if the robot can reach it's destination at
                              // target speed based on linear MAX acceleration
    bool b_can_stop_rotate_in_time;  // boolean value if the robot can reach it's
                                     // destination at target speed based on angular MAX
                                     // acceleration

    double robot_angular_velocity;
    const double distance_to_dest =
        (robot.position() - dest).len();  // destination distance used for constant linear
                                          // acceleration speed calculations
    const double angle_to_dest =
        (robot.orientation().toRadians() -
         desired_final_orientation.toRadians());  // rotation used for constant angular
                                                  // acceleration calculations

    // calculates robot angle based on unit vector that points from the robot location to
    // the destination (used to calculate the X/Y velocity magnitudes
    const Angle direction_angle = (dest - robot.position()).norm().orientation();

    // calculate the expected speed at the destination based on current speed and
    // acceleration Vf = sqrt( Vi^2 - 2*a*d)
    double expected_final_speed;
    double expected_final_ang_speed;

    // variables used to hold the change in velocities based on the maximum acceleration
    // and the change in time since the last motion controller run
    double delta_speed_x, delta_speed_y, delta_angular_speed;

    // check for negative sqrt case
    if (pow(robot.velocity().len(), 2) <= 2 * ROBOT_MAX_ACCELERATION * distance_to_dest)
    {
        // if the sqrt is negative, the final speed will be negative (opposite direction
        // of current speed)
        expected_final_speed = -1 * sqrt(2 * ROBOT_MAX_ACCELERATION * distance_to_dest -
                                         pow(robot.velocity().len(), 2));
        b_can_stop_in_time   = true;
    }
    else
    {
        // calculate the expected final speed assuming max decceleration
        expected_final_speed = sqrt(pow(robot.velocity().len(), 2) -
                                    2 * ROBOT_MAX_ACCELERATION * distance_to_dest);

        // the robot can stop in time if it's desired final speed is higher than the speed
        // if the robot maximum decelerates from the current state
        b_can_stop_in_time = expected_final_speed <= desired_final_speed;
    }

    // check negative sqrt case for angular speed
    if (pow(robot.angularVelocity().toRadians(), 2) <=
        fabs(2 * ROBOT_MAX_ANG_ACCELERATION * angle_to_dest))
    {
        expected_final_ang_speed =
            -1 * sqrt(fabs(2 * ROBOT_MAX_ANG_ACCELERATION * angle_to_dest) -
                      pow(robot.angularVelocity().toRadians(), 2));
        b_can_stop_rotate_in_time = true;
    }
    else
    {
        expected_final_ang_speed  = sqrt(pow(robot.angularVelocity().toRadians(), 2) -
                                        2 * ROBOT_MAX_ANG_ACCELERATION * angle_to_dest);
        b_can_stop_rotate_in_time = expected_final_ang_speed <= 0.0;
    }


    if (delta_time < 0)
    {  // there should be no negative changes in time except from the first run when
       // variables are initialized.
        delta_time = fabs(delta_time);
    }

    if (b_can_stop_in_time)
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

    // if the robot can stop rotating in time
    if (b_can_stop_rotate_in_time)
    {
        // if the final expected angle is less than the desired angle then angularly
        // accelerate
        if (expected_final_ang_speed < 0)
        {
            // angularly accelerate
            delta_angular_speed = ROBOT_MAX_ANG_ACCELERATION * delta_time;
        }

        // if not then maintain angular velocity
        else
        {
            delta_angular_speed = 0.0;
        }
    }

    // if the robot can't stop rotating in time then angular decelerate
    else
    {
        delta_angular_speed = -(ROBOT_MAX_ANG_ACCELERATION * delta_time);
    }

    robot_linear_velocities =
        Vector(robot.velocity().x() + delta_speed_x,
               robot.velocity().y() +
                   delta_speed_y);  // calculate new X/Y/ang velocities based on the
                                    // current robot speeds and delta speeds
    robot_angular_velocity =
        (robot.angularVelocity().toRadians()) +
        delta_angular_speed;  // calculate new angular velocity based on the current robot
                              // ang. velocity and delta ang. velocity

    if (robot_linear_velocities.len() > ROBOT_MAX_SPEED)
    {
        robot_linear_velocities = robot_linear_velocities.norm(ROBOT_MAX_SPEED);
    }

    if (fabs(robot_angular_velocity) > ROBOT_MAX_ANG_SPEED)
    {
        if (robot_angular_velocity < 0)
        {
            robot_angular_velocity = -1 * ROBOT_MAX_ANG_SPEED;
        }
        else
        {
            robot_angular_velocity = ROBOT_MAX_ANG_SPEED;
        }
    }

    return std::make_pair(robot_linear_velocities,
                          Angle::ofRadians(robot_angular_velocity));
}

#pragma once

#ifndef ROBOT_CONSTANTS_H
#define ROBOT_CONSTANTS_H

#include <math.h>

/** absolute angle to each of the front wheels as
 * measured from the front of the robots in radians
 * For 3rd generation robot 2015 CAD model
 * Last updated: Feb 3, 2018
 * /----------------\
 * |57.945 | -57.945|
 * |                |
 * |                |
 * |136.04 | -136.04|
 * \----------------/
 */
#define ANGLE_TO_FRONT_WHEELS 57.945f * (P_PI / 180.0f)
#define ANGLE_TO_BACK_WHEELS 136.04f * (P_PI / 180.0f)

// The number of wheels on the robot
#define NUMBER_OF_WHEELS 4

#endif

/**
 * This struct holds wheel/motor constants
 */
typedef struct WheelConstants
{
    // The current per unit torque for the motor attached to this wheel [A/(N*m)]
    float motor_current_per_unit_torque;

    // The phase resistance for the motor attached to this wheel [Ohms]
    float motor_phase_resistance;

    // The back emf per motor rpm for the motor attached to this wheel [volt / rpm]
    float motor_back_emf_per_rpm;

    // The maximum voltage change that can be exerted on the motor attached to this
    // wheel before the wheel will slip [Volts]
    float motor_max_voltage_before_wheel_slip;

    // The radius of the wheel, in meters
    float wheel_radius;

    // The gear ratio between the motor shaft and wheel shaft
    // [# of wheel rotations / 1 motor rotation]
    float wheel_rotations_per_motor_rotation;
} WheelConstants_t;

/**
 * This struct represents robot constants
 */
typedef struct RobotConstants
{
    // The mass of the entire robot including batteries [kg]
    // Determined experimentally by weighing the robot and battery
    float mass;

    // The moment of inertia of the entire robot [kg m^2]
    float moment_of_inertia;

    // The maximum radius of the robot [m]
    float robot_radius;

    // The maximum jerk this robot may safely undergo [m/s^3]
    float jerk_limit;

    // angle between each front wheel and the y axis of the robot
    float front_wheel_angle_deg;

    // angle between each back wheel and the y axis of the robot
    float back_wheel_angle_deg;

    // The total width of the entire flat face on the front of the robot
    float front_of_robot_width_meters;

    // The distance from one end of the dribbler to the other
    float dribbler_width_meters;

    // The maximum speed achievable by our robots, in metres per second.
    float robot_max_speed_meters_per_second;

    // The maximum angular speed achievable by our robots, in rad/sec
    float robot_max_ang_speed_rad_per_second;

    // The maximum acceleration achievable by our robots, in metres per seconds squared.
    float robot_max_acceleration_meters_per_second_squared;

    // The maximum angular acceleration achievable by our robots, in radians per second
    // squared
    float robot_max_ang_acceleration_rad_per_second_squared;

    // Indefinite dribbler mode sets a speed that can be maintained indefinitely
    float indefinite_dribbler_speed;

    // Max force dribbler mode sets the speed that applies the maximum amount of force on
    // the ball
    float max_force_dribbler_speed;
} RobotConstants_t;

/**
 * This struct holds the state of the controller.
 * This is a carryover from legacy code, and should be deleted when the controller is
 * replaced.
 */
typedef struct ControllerState
{
    float last_applied_acceleration_x;
    float last_applied_acceleration_y;
    float last_applied_acceleration_angular;
} ControllerState_t;

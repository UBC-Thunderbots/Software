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

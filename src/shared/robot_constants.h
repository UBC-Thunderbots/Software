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

typedef struct WheelAngles
{
    // Wheel angles for these matricies are (55, 135, 225, 305) degrees
    // these matrices may be derived as per omnidrive paper:
    // http://robocup.mi.fu-berlin.de/buch/omnidrive.pdf

    // This matrix is a unitless matrix which takes the force/speed exerted
    // on the floor by each of the four wheels and converts it into forces/speed
    // within the robot domain, the third component is is newtons and not
    // torque as the matrix is unitless (multiply by ROBOT_RADIUS to unnormalize)
    // the transpose of this matrix is the velocity coupling matrix and can
    // convert speeds in the robot coordinates into linear wheel speeds
    float shared_physics_wheels_to_local_vel_matrix_transpose[3][4];

    // Transformation matrix to convert a 4 velocity/force to a 3 velocity/force (derived
    // as pinv(shared_physics_force4ToForce3^t)
    float shared_physics_local_vel_to_wheels_matrix[3][4];
} WheelAngles_t;

/**
 * This struct represents robot constants
 */
typedef struct RobotConstants
{
    // The mass of the entire robot [kg]
    float mass;
    // The moment of inertia of the entire robot [kg m^2]
    float moment_of_inertia;
    // The maximum radius of the robot [m]
    float robot_radius;
    // The maximum jerk this robot may safely undergo [m/s^3]
    float jerk_limit;
    //    // angle of the wheels
    //    WheelAngles_t wheel_angles;
    //
    //    // TODO (#2066): The JERK_LIMIT is copied from firmware/main/control/control.h
    //    // which we currently can't include directly because it relies on firmware IO.
    //    // We should inject it as a robot or control param instead.
    //    float wheel_jerk_limit;  //(m/s^3)
    //    // TODO (#2066): The WHEEL_MOTOR_PHASE_RESISTANCE is copied from
    //    // firmware/main/io/wheels.h which we currently can't include directly because
    //    it is
    //    // in firmware IO. We should inject it as a robot or control param instead.
    //    float wheel_motor_phase_resistance;  // ohms—EC45 datasheet
    //
    //
    //    // The total width of the entire flat face on the front of the robot
    //    double front_of_robot_width_meters;
    //    // The distance from one end of the dribbler to the other
    //    double dribbler_width_meters;
    //
    //    /* Robot Attributes */
    //    // The mass of a robot with a battery, in kg. Determined experimentally
    //    // by weighing the robot and battery
    //    double robot_with_battery_mass_kg;
    //    // The maximum speed achievable by our robots, in metres per second.
    //    double robot_max_speed_meters_per_second;
    //    // The maximum angular speed achievable by our robots, in rad/sec
    //    double robot_max_ang_speed_rad_per_second;
    //    // The maximum acceleration achievable by our robots, in metres per seconds
    //    squared.
    //    double robot_max_acceleration_meters_per_second_squared;
    //
    //    // The maximum angular acceleration achievable by our robots, in radians per
    //    second squared
    //    double robot_max_ang_acceleration_rad_per_second_squared;
    //
    //    // Indefinite dribbler mode sets a speed that can be maintained indefinitely
    //    double indefinite_dribbler_speed;
    //    // Max force dribbler mode sets the speed that applies the maximum amount of
    //    force on
    //    // the ball
    //    double max_force_dribbler_speed;
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

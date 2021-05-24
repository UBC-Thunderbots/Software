#pragma once

#include "firmware/shared/math/tbots_math.h"

// This file contains all the physical constants of the robot
// Dimensions and the like as well as

#define CONTROL_LOOP_HZ 200U
#define QUARTERDEGREE_TO_MS (0.0000554f * CONTROL_LOOP_HZ)
#define QUARTERDEGREE_TO_RPM                                                             \
    (CONTROL_LOOP_HZ / 240.0f)       // encoder quarter of degree to motor RPM
#define RPM_TO_VOLT (1.0f / 374.0f)  // motor RPM to back EMF

#define HALL_PHASE_TO_MS (0.00171 * CONTROL_LOOP_HZ)

#define QUARTERDEGREE_TO_VOLT (QUARTERDEGREE_TO_RPM * RPM_TO_VOLT)

#define ROBOT_RADIUS 0.085f
#define TICK_TIME (1.0f / (float)CONTROL_LOOP_HZ)
#define ROBOT_POINT_MASS 2.48f
#define WHEEL_SLIP_VOLTAGE_LIMIT 4.25f  // Voltage where wheel slips (acceleration cap)

// Maximum safe jerk for the robot
#define JERK_LIMIT 40.0f  //(m/s^3)

// all the interial components of the robot
// This one is a little strange as it is the effective rotational mass
// The rotational mass * (ROBOT_RADIUS)^2 will give the conventional interia
#define INERTIAL_FACTOR 0.37f

// factor for steel motor mounts
#define STEEL_INTERTIAL_FACTOR 0.3858f

#define ROT_MASS (INERTIAL_FACTOR * ROBOT_POINT_MASS)
#define INERTIA (ROT_MASS * ROBOT_RADIUS * ROBOT_RADIUS)

#define CURRENT_PER_TORQUE 39.21f  // from motor data sheet (1/25.5 mNm)
#define GEAR_RATIO 0.5143f         // define as speed multiplication from motor to wheel
#define WHEEL_RADIUS 0.0254f

// constants for radial bangbang controller
#define MAX_R_V 2.0f
#define MAX_R_A 3.0f


#define MAX_X_V 2.0f  // maximal linear velocity in the X direction
#define MAX_Y_V 1.0f  // maximum linear velocity in the Y direction
#define MAX_T_V 0.1f  // max robot rotation rate in radians per second

extern const float MAX_VEL[3];

// WRONG NUMBERS, POKE JON TO GET ACTUAL NUMBERS
#define MAX_X_A 3.0f
#define MAX_Y_A 3.0f
#define MAX_T_A 30.0f

//#define SLOW_MAX_X_A 1.2f
//#define SLOW_MAX_Y_A 1.2f
//#define SLOW_MAX_T_A 10.0f

extern const float MAX_ACC[3];

// gyro running at 2000/second and in integers such that 32767 is 2000
// 61.0 millidegrees/second / LSB
#define DEGREES_PER_GYRO (61.0f / 1000.0f)
#define MS_PER_DEGREE (2.0f * (float)P_PI * ROBOT_RADIUS / 360.0f)
#define MS_PER_GYRO (MS_PER_DEGREE * DEGREES_PER_GYRO)

// Accelerometer is running at +/- 2G's, 32767 is 2G's
// ~61.0 uG per accelerometer LSB
#define GRAVITY 9.807f
#define ACCEL_RANGE 2.0f
#define NUM_DIVISIONS 32767
#define G_PER_ACCEL (ACCEL_RANGE / NUM_DIVISIONS)
#define M_S_2_PER_ACCEL (GRAVITY * G_PER_ACCEL)

extern const float ROBOT_MASS[3];
extern const float MAX_VEL[3];


// transformation matricies to convert speeds in the
// two different domains commonly used by the robot
// speed4 which is the listing of wheel speeds
// and speed3 which is a speed in x,y,rotation in
// robot relative coordinates
void shared_physics_speed4ToSpeed3(const float speed4[4], float speed3[3]);
void shared_physics_speed3ToSpeed4(const float speed3[3], float speed4[4]);

// transformation matricies to convert forces in the
// two different domains commonly used by the robot
// force4 which is the listing of wheel forces
// and force3 which is a force in x,y,rotation in
// robot relative coordinates
void shared_physics_force3ToForce4(float force3[3], float force4[4]);

float shared_physics_minAngleDelta(float, float);

float shared_physics_norm2(float a1, float a2);

// rotate a velocity vector through angle
void shared_physics_rotate(float speed3[2], float angle);

float shared_physics_dotProduct(const float vec1[], const float vec2[], const int size);

float shared_physics_dot2D(float vec1[2], float vec2[2]);

/**
 * Function calculates the final speed at the end of a displacement given an initial speed
 * and constant acceleration
 *
 * Source:
 * https://www.khanacademy.org/science/physics/one-dimensional-motion/kinematic-formulas/a/what-are-the-kinematic-formulas
 *
 * @pre All values are positive
 *
 * @param initial_speed The initial speed in meters/second
 * @param displacement The total displacement accelerated over in meters
 * @param acceleration The constant acceleration over the displacement in
 * meters/second^2
 *
 * @return The final speed in meters/second
 */
float shared_physics_getFinalSpeed(float initial_speed, float displacement,
                                   float acceleration);

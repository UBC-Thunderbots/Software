#pragma once

#include "firmware/shared/math/tbots_math.h"
#include "shared/constants.h"

// This file contains all the physical constants of the robot
// Dimensions and the like as well as

#define CONTROL_LOOP_HZ 200U
#define QUARTERDEGREE_TO_MS (0.0000554f * CONTROL_LOOP_HZ)
#define QUARTERDEGREE_TO_RPM                                                             \
    (CONTROL_LOOP_HZ / 240.0f)  // encoder quarter of degree to motor RPM

#define HALL_PHASE_TO_MS (0.00171 * CONTROL_LOOP_HZ)

#define QUARTERDEGREE_TO_VOLT (QUARTERDEGREE_TO_RPM * RPM_TO_VOLT)

#define TICK_TIME (1.0f / (float)CONTROL_LOOP_HZ)

// factor for steel motor mounts
#define STEEL_INTERTIAL_FACTOR 0.3858f

/**
 * Convert a set of given wheel speeds into linear x,y, and angular rotation speed or vice
 * versa. All arguments are unitless, i.e. the same units in and out, and don't depend on
 * the wheel radius (i.e. angular speed)
 *
 * @param [in] speed4 which is the listing of wheel speeds, positive spins the wheel
 * counter-clockwise, as viewed from inside the robot looking out
 * @param [out] speed3 which is a speed in x,y,rotation in
 * robot relative coordinates
 * @param front_wheel_angle_deg absolute angle between each front wheel and the y axis of
 * the robot in degrees
 * @param back_wheel_angle_deg absolute angle between each back wheel and the y axis of
 * the robot in degrees
 */
void shared_physics_speed4ToSpeed3(const float speed4[4], float speed3[3],
                                   float front_wheel_angle_deg,
                                   float back_wheel_angle_deg);
void shared_physics_speed3ToSpeed4(const float speed3[3], float speed4[4],
                                   float front_wheel_angle_deg,
                                   float back_wheel_angle_deg);

/**
 * Convert a set of given wheel speeds into linear x,y, and angular rotation speeds. All
 * arguments are unitless, i.e. the same units in and out, and don't depend on the wheel
 * radius (i.e. angular speed)
 *
 * NOTE: this legacy conversion function is deprecated and  assumes that the angles are
 * 55deg (front) and 45deg (back). Please use shared_physics_speed4ToSpeed3 instead
 *
 * @param [in] speed4 which is the listing of wheel speeds, positive spins the wheel
 * counter-clockwise, as viewed from inside the robot looking out
 * @param [out] speed3 which is a speed in x,y,rotation in
 * robot relative coordinates
 */
void shared_physics_legacySpeed4ToSpeed3(const float speed4[4], float speed3[3]);

/**
 * Convert linear x,y, and angular rotation forces into a set of given wheel forces. All
 * arguments are unitless, i.e. the same units in and out, and don't depend on the wheel
 * radius (i.e. angular force)
 *
 * @param [in] force4 which is the listing of wheel forces, positive spins the wheel
 * counter-clockwise, as viewed from inside the robot looking out
 * @param [out] force3 which is a force in x,y,rotation in
 * robot relative coordinates
 * @param front_wheel_angle_deg absolute angle between each front wheel and the y axis of
 * the robot in degrees
 * @param back_wheel_angle_deg absolute angle between each back wheel and the y axis of
 * the robot in degrees
 */
void shared_physics_force3ToForce4(float force3[3], float force4[4],
                                   float front_wheel_angle_deg,
                                   float back_wheel_angle_deg);

/**
 * Minimum difference between the two angles
 *
 * @param angle1 the first angle in radians
 * @param angle2 the second angle in radians
 *
 * @return the min angle difference in radians
 */
float shared_physics_minAngleDelta(float angle1, float angle2);

/**
 * Calculates the l2 norm between a1 and a2
 *
 * @param a1 The first number
 * @param a2 The second number
 *
 * @return the l2 norm between the 2 numbers
 */
float shared_physics_norm2(float a1, float a2);

/**
 * rotate a velocity vector through angle
 *
 * @param speed3 [in/out] the vector to rotate
 * @param angle the angle to rotate through in radians
 */
void shared_physics_rotate(float speed3[2], float angle);

/**
 * Calculates the dot product between two vectors of the same size
 *
 * @param vec1 the first vector
 * @param vec2 the second vector
 * @param size the size of the vectors
 *
 * @return the dot product
 */
float shared_physics_dotProduct(const float vec1[], const float vec2[], const int size);

/**
 * Calculates the 2D dot product between two matrices of the same size
 *
 * @param vec1 the first vector
 * @param vec2 the second vector
 * @param size the size of the vectors
 *
 * @return the dot product
 */
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

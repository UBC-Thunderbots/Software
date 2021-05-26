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

#define TICK_TIME (1.0f / (float)CONTROL_LOOP_HZ)
#define WHEEL_SLIP_VOLTAGE_LIMIT 4.25f  // Voltage where wheel slips (acceleration cap)

// factor for steel motor mounts
#define STEEL_INTERTIAL_FACTOR 0.3858f

#define CURRENT_PER_TORQUE 39.21f  // from motor data sheet (1/25.5 mNm)
#define GEAR_RATIO 0.5143f         // define as speed multiplication from motor to wheel
#define WHEEL_RADIUS 0.0254f

/**
 * Transformation matricies to convert speeds in the
 * two different domains commonly used by the robot
 *
 * @param [in] speed4 which is the listing of wheel speeds
 * @param [out] speed3 which is a speed in x,y,rotation in
 * robot relative coordinates
 * @param front_wheel_angle_deg angle between each front wheel and the y axis of the robot
 * in degrees
 * @param back_wheel_angle_deg angle between each back wheel and the y axis of the robot
 * in degrees
 */
void shared_physics_speed4ToSpeed3(const float speed4[4], float speed3[3],
                                   float front_wheel_angle_deg,
                                   float back_wheel_angle_deg);
void shared_physics_speed3ToSpeed4(const float speed3[3], float speed4[4],
                                   float front_wheel_angle_deg,
                                   float back_wheel_angle_deg);

/**
 * Transformation matricies to convert forces in the
 * two different domains commonly used by the robot
 *
 * @param [in] force4 which is the listing of wheel forces
 * @param [out] force3 which is a force in x,y,rotation in
 * robot relative coordinates
 * @param front_wheel_angle_deg angle between each front wheel and the y axis of the robot
 * in radians
 * @param back_wheel_angle_deg angle between each back wheel and the y axis of the robot
 * in radians
 */
void shared_physics_force3ToForce4(float force3[3], float force4[4],
                                   float front_wheel_angle_deg,
                                   float back_wheel_angle_deg);

/**
 * Min angle delta between the radians of two angles
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

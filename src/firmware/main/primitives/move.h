#ifndef PRIMITIVES_MOVE_H
#define PRIMITIVES_MOVE_H

#include "physics/physics.h"
#include "primitive.h"
#include "util/physbot.h"


extern const primitive_t MOVE_PRIMITIVE;

// TODO: Find out actual wheel angle
// this should be the angle between the front of the bot and either
// the closest right or left wheel
static const float CLOSEST_WHEEL_ANGLE = 30.0f * P_PI / 180.0f;

// The minimum distance away from our destination that we must be if we
// are going to rotate the bot onto its wheel axis
// 2 * P_PI * ROBOT_RADIUS = robot circumference, which is approximately
// how far the bot would have to turn for one full rotation, so we
// set it a litle larger than that.
static const float APPROACH_LIMIT = 3 * P_PI * ROBOT_RADIUS;

#define VAL_EQUIVALENT_2_ZERO (5e-3f)
#define CONTROL_TICK (1.0f / CONTROL_LOOP_HZ)

#define LOOK_AHEAD_T 10

/**
 * call from move_start to choose which wheel axis we will be
 * using for preliminary rotation. The idea is to pick the wheel
 * axis that will result in the minimum remaining rotation onto
 * the bot's final destination angle.
 *
 * @param dx the global x position of the bot
 * @param dy the global y position of the bot
 * @param current_angle the current angle of the bot
 * @param final_angle the final destination angle
 * @return the index of the wheel axis to use
 */
unsigned choose_wheel_axis(float dx, float dy, float current_angle, float final_angle);

/**
 * Calculates the rotation time, velocity, and acceleration to be stored
 * in a PhysBot data container.
 *
 * @param pb The data container that has information about major axis time
 * and will store the rotational information
 * @param avel The current rotational velocity of the bot
 * @return void
 */
void plan_move_rotation(PhysBot *pb, float avel);

#endif

#pragma once

#include "firmware/app/world/firmware_world.h"

/**
 * Chick motion figures out how to line the robot up for a kick or chip, but it is up
 * to users of these helper functions to arm the appropriate autokick and autochip
 */

/**
 * <------- B <-------------------------
 *        ^       ^                 ^         \
 * alignment   destination   chick motion path \
 *   angle                                      \
 *                                               \
 *                                                A
 *                                                ^
 *                                              start
 */

/**
 * Start the chick motion in the given world
 *
 * @param void_state_ptr [in] A pointer to this primitives state, as allocated by the
 *                            `create_state` function
 * @param world [in] The world this primitive is running in.
 * @param x_destination The x destination for shooting
 * @param y_destination The y destination for shooting
 * @param alignment_angle The angle to align to
 */
void app_chick_motion_start(void *void_state_ptr, FirmwareWorld_t *world,
                            float x_destination, float y_destination,
                            float alignment_angle);


// The following function definitions mirror those in primitive_t
void app_chick_motion_tick(void *void_state_ptr, FirmwareWorld_t *world);
void *createChickMotionState_t(void);
void destroyChickMotionState_t(void *state);

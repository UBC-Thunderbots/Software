#pragma once

#include "firmware/app/world/firmware_world.h"

/**
 * Align to ball figures out how to line the robot up for a kick or chip, but it is up
 * to users of these helper functions to arm the appropriate autokick and autochip
 */

/**
 * Start the align to ball in the given world
 *
 * @param void_state_ptr [in] A pointer to this primitives state, as allocated by the
 *                            `create_state` function
 * @param world [in] The world this primitive is running in.
 * @param x_destination The x destination for shooting
 * @param y_destination The y destination for shooting
 * @param alignment_angle The angle to align to
 */
void app_align_to_ball_start(void *void_state_ptr, FirmwareWorld_t *world,
                             float x_destination, float y_destination,
                             float alignment_angle);


// The following function definitions mirror those in primitive_t
void app_align_to_ball_end(void *void_state_ptr, FirmwareWorld_t *world);
void app_align_to_ball_tick(void *void_state_ptr, FirmwareWorld_t *world);
void *createAlignToBallState_t(void);
void destroyAlignToBallState_t(void *state);

#pragma once

#include "firmware/app/world/firmware_world.h"

/**
 * Start the shoot alignment in the given world
 * @param void_state_ptr [in] A pointer to this primitives state, as allocated by the
 *                            `create_state` function
 * @param world [in] The world this primitive is running in.
 * @param x_destination The x destination for shooting
 * @param y_destination The y destination for shooting
 * @param alignment_angle The angle to align to
 */
void app_shoot_alignment_start(void *void_state_ptr, FirmwareWorld_t *world,
                               float x_destination, float y_destination,
                               float alignment_angle);

// The following function definitions mirror those in primitive_t
void app_shoot_alignment_end(void *void_state_ptr, FirmwareWorld_t *world);
void app_shoot_alignment_tick(void *void_state_ptr, FirmwareWorld_t *world);
void *createShootAlignmentState_t(void);
void destroyShootAlignmentState_t(void *state);

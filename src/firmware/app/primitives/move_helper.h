#pragma once

#include <stdbool.h>

#include "firmware/app/world/firmware_world.h"
#include "shared/proto/primitive.nanopb.h"

/**
 * Start moving in the given world
 * @param void_state_ptr [in] A pointer to this primitives state, as allocated by the
 *                            `create_state` function
 * @param world [in] The world this primitive is running in.
 * @param move_position_params Move Position Params
 * @param final_angle The final angle
 */
void app_move_helper_start(void *void_state_ptr, FirmwareWorld_t *world,
                           MovePositionParams move_position_params, float final_angle);

// The following function definitions mirror those in primitive_t
void app_move_helper_end(void *void_state_ptr, FirmwareWorld_t *world);
void app_move_helper_tick(void *void_state_ptr, FirmwareWorld_t *world);
void *createMoveHelperState_t(void);
void destroyMoveHelperState_t(void *state);

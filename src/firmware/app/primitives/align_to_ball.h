#pragma once

#include "primitive.h"

/**
 * Align to ball figures out how to line the robot up for a kick or chip, but it is up
 * to users of these helper functions to arm the appropriate autokick and autochip. Align
 * to ball uses the primitive structure for ease of use by other primitives, but it is not
 * meant to be used as a standalone primitive
 */
extern const primitive_t ALIGN_TO_BALL;

/**
 * Start the align to ball in the given world
 *
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

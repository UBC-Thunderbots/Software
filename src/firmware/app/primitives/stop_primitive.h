#pragma once

#include "firmware/app/primitives/primitive.h"

extern const primitive_t STOP_PRIMITIVE;

/**
 * Start the Stop primitive in the given world
 *
 * @param prim_msg The msg for the primitive
 * @param void_state_ptr [in] A pointer to this primitives state, as allocated by the
 *                            `create_state` function
 * @param world [in] The world this primitive is running in.
 */
void app_stop_primitive_start(TbotsProto_StopPrimitive prim_msg, void* void_state_ptr,
                              FirmwareWorld_t* world);

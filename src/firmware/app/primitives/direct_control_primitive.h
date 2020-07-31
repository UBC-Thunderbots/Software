#pragma once
#include "firmware/app/primitives/primitive.h"
#include "shared/proto/primitive.nanopb.h"

extern const primitive_t DIRECT_CONTROL_PRIMITIVE;

/**
 * Start the DirectControlPrimitive in the given world
 *
 * @param prim_msg [in] The proto msg for this primitive
 * @param void_state_ptr [in] A pointer to this primitives state, as allocated by the
 *                            `create_state` function
 * @param world [in] The world this primitive is running in.
 */
void app_direct_control_primitive_start(DirectControlPrimitiveMsg prim_msg,
                                        void* void_state_ptr, FirmwareWorld_t* world);

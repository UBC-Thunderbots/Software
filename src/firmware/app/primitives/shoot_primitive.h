#pragma once

#include "primitive.h"
#include "shared/proto/primitive.nanopb.h"

extern const primitive_t SHOOT_PRIMITIVE;

/**
 * Start the Shoot primitive in the given world
 * @param params [in] The params for the primitive
 * @param void_state_ptr [in] A pointer to this primitives state, as allocated by the
 *                            `create_state` function
 * @param world [in] The world this primitive is running in.
 */
void app_shoot_primitive_start(PrimitiveParamsMsg params, void* void_state_ptr,
                               FirmwareWorld_t* world);

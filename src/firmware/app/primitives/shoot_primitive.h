#pragma once

#include "primitive.h"
#include "shared/proto/primitive.nanopb.h"

extern const primitive_t SHOOT_PRIMITIVE;

void app_shoot_primitive_start(PrimitiveParamsMsg params, void* void_state_ptr,
                               FirmwareWorld_t* world);

#pragma once
#include "firmware/app/primitives/primitive.h"
#include "shared/proto/primitive.nanopb.h"

extern const primitive_t STOP_PRIMITIVE;

void app_stop_primitive_start(PrimitiveParamsMsg params, void* void_state_ptr,
                              FirmwareWorld_t* world);

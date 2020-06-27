#pragma once
#include "firmware/app/primitives/primitive.h"
#include "shared/proto/primitive.pb.h"

extern const primitive_t SPIN_PRIMITIVE;

void app_spin_primitive_start(PrimitiveParamsMsg params, void* void_state_ptr,
                               FirmwareWorld_t* world);

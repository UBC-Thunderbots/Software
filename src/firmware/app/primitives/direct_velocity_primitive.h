#pragma once
#include "firmware/app/primitives/primitive.h"
#include "shared/proto/primitive.pb.h"

extern const primitive_t DIRECT_VELOCITY_PRIMITIVE;

// TODO: jdoc
void app_direct_velocity_primitive_start(PrimitiveParamsMsg params, void* void_state_ptr,
                               FirmwareWorld_t* world);

#pragma once
#include "firmware/app/primitives/primitive.h"
#include "shared/proto/primitive.pb.h"

extern const primitive_t DIRECT_WHEELS_PRIMITIVE;

// TODO: jdoc
void app_direct_wheels_primitive_start(PrimitiveParamsMsg params, void* void_state_ptr,
                               FirmwareWorld_t* world);

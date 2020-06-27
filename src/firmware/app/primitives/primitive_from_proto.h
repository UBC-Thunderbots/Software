#pragma once

#include "firmware/app/primitives/primitive.h"
#include "shared/proto/primitive.pb.h"

// TODO: jdoc
bool create_firmware_primitive_from_proto(PrimitiveMsg proto, primitive_t* created_primitive);
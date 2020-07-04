#pragma once

#include "software/simulation/serialize_primitive_to_proto.h"
#include "software/primitive/primitive.h"

extern "C" {
#include "shared/proto/primitive.nanopb.h"
}

// TODO: better name?
// TODO: jdoc?
PrimitiveMsg convertPrimitiveToNanoPb(const Primitive& primitive);
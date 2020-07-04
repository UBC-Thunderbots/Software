#pragma once

// TODO: large doc comment here and at it's usage on why it exists

#include <vector>
#include "software/primitive/primitive.h"

// TODO: better name for this?
// TODO: jdoc for this
// TODO: test this?
std::vector<uint8_t> serializePrimitiveToProto(const Primitive& primitive);

#pragma once

#include <vector>
#include "software/primitive/primitive.h"

/**
 * Serialize the given primitive using Google protobuf
 *
 * IMPORTANT NOTE: The reason this function is in it's own file is because we can't
 *     include NanoPb and Google Proto headers in the same `.cpp`, as they define the
 *     same names, and so will have naming conflicts and not compile. So we have
 *     the Primitive -> Google Proto -> Serialized Data in this file, so elsewhere
 *     we can do the Serialized Data -> NanoPb without ever "knowing" about Google Proto
 *
 * @param primitive The primitive to serialize
 * @return The serialized primitive
 */
std::vector<uint8_t> serializePrimitiveToProto(const Primitive& primitive);

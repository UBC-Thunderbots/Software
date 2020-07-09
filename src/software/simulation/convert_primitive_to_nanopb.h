#pragma once

#include "software/primitive/primitive.h"

extern "C"
{
#include "shared/proto/primitive.nanopb.h"
}

/**
 * Convert the given primitive to a NanoPb message
 *
 * @param primitive The primitive to convert to a NanoPb message
 * @return The NanoPb message representing the given primitive
 */
PrimitiveMsg createNanoPbPrimitiveMsg(const Primitive& primitive);

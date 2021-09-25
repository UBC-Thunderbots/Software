#pragma once

#include "proto/primitive.pb.h"
#include "proto/tbots_software_msgs.pb.h"

extern "C"
{
#include "proto/primitive.nanopb.h"
#include "proto/tbots_software_msgs.nanopb.h"
}

/**
 * Convert the given google primitive proto to a NanoPb message
 *
 * @param google_primitive The google primitive proto to convert to a NanoPb message
 *
 * @return The NanoPb message representing the given primitive
 */
TbotsProto_Primitive createNanoPbPrimitive(const TbotsProto::Primitive& google_primitive);

/**
 * Convert the given google primitive set proto to a NanoPb message
 *
 * @param google_primitive_set The google primitive set proto to convert to a NanoPb
 * message
 *
 * @return The NanoPb message representing the given primitive
 */
TbotsProto_PrimitiveSet createNanoPbPrimitiveSet(
    const TbotsProto::PrimitiveSet& google_primitive_set);

#include "software/simulation/convert_primitive_to_nanopb.h"

#include <pb_decode.h>

#include "software/simulation/serialize_primitive_to_proto.h"

PrimitiveMsg createNanoPbPrimitiveMsg(const Primitive& primitive)
{
    std::vector<uint8_t> serialized_proto = serializePrimitiveToProto(primitive);

    PrimitiveMsg primitive_msg = PrimitiveMsg_init_zero;

    pb_istream_t pb_in_stream = pb_istream_from_buffer(
        static_cast<uint8_t*>(serialized_proto.data()), serialized_proto.size());
    if (!pb_decode(&pb_in_stream, PrimitiveMsg_fields, &primitive_msg))
    {
        throw std::runtime_error(
            "Failed to decode serialized primitive to NanoPb when converting Primitive class to NanoPb");
    }

    return primitive_msg;
}

#include "software/simulation/convert_primitive_to_nanopb.h"

#include <pb_decode.h>

#include "software/simulation/serialize_primitive_to_proto.h"

TbotsProto_Primitive createNanoPbTbotsProto_Primitive(const Primitive& primitive)
{
    std::vector<uint8_t> serialized_proto = serializePrimitiveToProto(primitive);

    TbotsProto_Primitive primitive_msg = TbotsProto_Primitive_init_zero;

    pb_istream_t pb_in_stream = pb_istream_from_buffer(
        static_cast<uint8_t*>(serialized_proto.data()), serialized_proto.size());
    if (!pb_decode(&pb_in_stream, TbotsProto_Primitive_fields, &primitive_msg))
    {
        throw std::runtime_error(
            "Failed to decode serialized primitive to NanoPb when converting Primitive class to NanoPb");
    }

    return primitive_msg;
}

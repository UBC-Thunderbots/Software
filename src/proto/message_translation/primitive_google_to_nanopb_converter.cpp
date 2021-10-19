#include "proto/message_translation/primitive_google_to_nanopb_converter.h"

#include <pb_decode.h>

TbotsProto_Primitive createNanoPbPrimitive(const TbotsProto::Primitive& google_primitive)
{
    // Serialize the message to an array of raw values
    std::vector<uint8_t> serialized_proto(google_primitive.ByteSizeLong());
    google_primitive.SerializeToArray(serialized_proto.data(),
                                      static_cast<int>(google_primitive.ByteSizeLong()));


    TbotsProto_Primitive nanopb_primitive = TbotsProto_Primitive_init_zero;

    pb_istream_t pb_in_stream = pb_istream_from_buffer(
        static_cast<uint8_t*>(serialized_proto.data()), serialized_proto.size());
    if (!pb_decode(&pb_in_stream, TbotsProto_Primitive_fields, &nanopb_primitive))
    {
        throw std::runtime_error(
            "Failed to decode serialized primitive to NanoPb when converting google Primitive proto to NanoPb");
    }

    return nanopb_primitive;
}

TbotsProto_PrimitiveSet createNanoPbPrimitiveSet(
    const TbotsProto::PrimitiveSet& google_primitive_set)
{
    // Serialize the message to an array of raw values
    std::vector<uint8_t> serialized_proto(google_primitive_set.ByteSizeLong());
    google_primitive_set.SerializeToArray(
        serialized_proto.data(), static_cast<int>(google_primitive_set.ByteSizeLong()));


    TbotsProto_PrimitiveSet nanopb_primitive_set = TbotsProto_PrimitiveSet_init_zero;

    pb_istream_t pb_in_stream = pb_istream_from_buffer(
        static_cast<uint8_t*>(serialized_proto.data()), serialized_proto.size());
    if (!pb_decode(&pb_in_stream, TbotsProto_PrimitiveSet_fields, &nanopb_primitive_set))
    {
        throw std::runtime_error(
            "Failed to decode serialized primitive_set to NanoPb when converting google PrimitiveSet proto to NanoPb");
    }

    return nanopb_primitive_set;
}

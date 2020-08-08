#include "software/simulation/serialize_primitive_to_proto.h"

#include "software/proto/message_translation/proto_creator_primitive_visitor.h"

std::vector<uint8_t> serializePrimitiveToProto(const Primitive& primitive)
{
    TbotsProto::Primitive primitive_msg =
        ProtoCreatorPrimitiveVisitor().createPrimitive(primitive);

    // Serialize the message to an array of raw values
    std::vector<uint8_t> serialized_proto(primitive_msg.ByteSizeLong());
    primitive_msg.SerializeToArray(serialized_proto.data(),
                                   static_cast<int>(primitive_msg.ByteSizeLong()));

    return serialized_proto;
}

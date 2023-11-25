#include "software/ai/hl/stp/tactic/stop_primitive.h"

std::unique_ptr<TbotsProto::Primitive> StopPrimitive::generatePrimitiveProtoMessage()
{
    auto stop_primitive_msg = std::make_unique<TbotsProto::Primitive>();
    stop_primitive_msg->mutable_stop();
    return stop_primitive_msg;
}

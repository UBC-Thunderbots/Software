#include "software/ai/intent/direct_primitive_intent.h"

#include <google/protobuf/util/message_differencer.h>

DirectPrimitiveIntent::DirectPrimitiveIntent(unsigned int robot_id,
                                             TbotsProto::Primitive primitive_msg)
    : Intent(robot_id), primitive_msg(primitive_msg)
{
}

void DirectPrimitiveIntent::accept(IntentVisitor &visitor) const
{
    visitor.visit(*this);
}

bool DirectPrimitiveIntent::operator==(const DirectPrimitiveIntent &other) const
{
    return Intent::operator==(other) &&
           google::protobuf::util::MessageDifferencer::Equals(this->primitive_msg,
                                                              other.primitive_msg);
}

bool DirectPrimitiveIntent::operator!=(const DirectPrimitiveIntent &other) const
{
    return !((*this) == other);
}

TbotsProto::Primitive DirectPrimitiveIntent::getPrimitive() const
{
    return primitive_msg;
}

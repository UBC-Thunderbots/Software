#include "software/ai/intent/stop_intent.h"

#include <google/protobuf/util/message_differencer.h>

const std::string StopIntent::INTENT_NAME = "Stop Intent";

StopIntent::StopIntent(unsigned int robot_id, bool coast, unsigned int priority)
    : Intent(robot_id, priority),
      primitive_msg(ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(
          StopPrimitive(robot_id, coast)))
{
}

std::string StopIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

void StopIntent::accept(IntentVisitor &visitor) const
{
    visitor.visit(*this);
}

bool StopIntent::operator==(const StopIntent &other) const
{
    return Intent::operator==(other) &&
           google::protobuf::util::MessageDifferencer::Equals(this->primitive_msg,
                                                              other.primitive_msg);
}

bool StopIntent::operator!=(const StopIntent &other) const
{
    return !((*this) == other);
}

PrimitiveMsg StopIntent::generatePrimitive() const
{
    return primitive_msg;
}

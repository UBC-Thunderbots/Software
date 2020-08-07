#include "software/ai/intent/spinning_move_intent.h"

#include <google/protobuf/util/message_differencer.h>

const std::string SpinningMoveIntent::INTENT_NAME = "Spinning Move Intent";

SpinningMoveIntent::SpinningMoveIntent(unsigned int robot_id, const Point &dest,
                                       const AngularVelocity &angular_vel,
                                       double final_speed, unsigned int priority)
    : Intent(robot_id, priority),
      primitive_msg(ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(
          SpinningMovePrimitive(robot_id, dest, angular_vel, final_speed)))
{
}

std::string SpinningMoveIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

void SpinningMoveIntent::accept(IntentVisitor &visitor) const
{
    visitor.visit(*this);
}

bool SpinningMoveIntent::operator==(const SpinningMoveIntent &other) const
{
    return Intent::operator==(other) &&
           google::protobuf::util::MessageDifferencer::Equals(this->primitive_msg,
                                                              other.primitive_msg);
}

bool SpinningMoveIntent::operator!=(const SpinningMoveIntent &other) const
{
    return !((*this) == other);
}

PrimitiveMsg SpinningMoveIntent::generatePrimitive() const
{
    return primitive_msg;
}

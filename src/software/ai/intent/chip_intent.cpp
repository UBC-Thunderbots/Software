#include "software/ai/intent/chip_intent.h"

#include <google/protobuf/util/message_differencer.h>

const std::string ChipIntent::INTENT_NAME = "Chip Intent";

ChipIntent::ChipIntent(unsigned int robot_id, const Point &chip_origin,
                       const Angle &chip_direction, double chip_distance_meters,
                       unsigned int priority)
    : Intent(robot_id, priority),
      primitive_msg(ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(
          ChipPrimitive(robot_id, chip_origin, chip_direction, chip_distance_meters)))
{
}

std::string ChipIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

bool ChipIntent::operator==(const ChipIntent &other) const
{
    return Intent::operator==(other) &&
           google::protobuf::util::MessageDifferencer::Equals(this->primitive_msg,
                                                              other.primitive_msg);
}

bool ChipIntent::operator!=(const ChipIntent &other) const
{
    return !((*this) == other);
}

PrimitiveMsg ChipIntent::generatePrimitiveMsg()
{
    return primitive_msg;
}

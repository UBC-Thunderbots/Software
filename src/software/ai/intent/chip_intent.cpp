#include "software/ai/intent/chip_intent.h"

const std::string ChipIntent::INTENT_NAME = "Chip Intent";

ChipIntent::ChipIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed, unsigned int priority)
    : ChipPrimitive(robot_id, dest, final_angle, final_speed), Intent(priority)
{
}

std::string ChipIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

void ChipIntent::accept(IntentVisitor &visitor) const
{
    visitor.visit(*this);
}

bool ChipIntent::operator==(const ChipIntent &other) const
{
    return ChipPrimitive::operator==(other) && Intent::operator==(other);
}

bool ChipIntent::operator!=(const ChipIntent &other) const
{
    return !((*this) == other);
}

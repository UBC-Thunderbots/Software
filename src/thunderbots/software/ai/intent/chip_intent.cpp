#include "chip_intent.h"

const std::string ChipIntent::INTENT_NAME = "Chip Intent";

ChipIntent::ChipIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed)
    : ChipPrimitive(robot_id, dest, final_angle, final_speed)
{
}

std::string ChipIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

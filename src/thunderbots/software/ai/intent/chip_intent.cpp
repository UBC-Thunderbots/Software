#include "chip_intent.h"

ChipIntent::ChipIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed, int priority)
    : ChipPrimitive(robot_id, dest, final_angle, final_speed), priority(priority) 
{
}


ChipIntent::ChipIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed)
    : ChipPrimitive(robot_id, dest, final_angle, final_speed)
{
}

std::string ChipIntent::getIntentName() const
{
    return CHIP_INTENT_NAME;
}

int ChipIntent::getPriority() const
{
    return priority;
}

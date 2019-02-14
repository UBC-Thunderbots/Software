#include "move_intent.h"

const std::string MoveIntent::INTENT_NAME = "Move Intent";

MoveIntent::MoveIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed)
    : MovePrimitive(robot_id, dest, final_angle, final_speed)
{
}

std::string MoveIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

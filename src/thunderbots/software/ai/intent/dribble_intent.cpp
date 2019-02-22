#include "ai/intent/dribble_intent.h"

const std::string DribbleIntent::INTENT_NAME = "Dribble Intent";

DribbleIntent::DribbleIntent(unsigned int robot_id, const Point &dest,
                             const Angle &final_angle, double final_speed, double rpm,
                             bool small_kick_allowed, unsigned int priority)
    : DribblePrimitive(robot_id, dest, final_angle, final_speed, rpm, small_kick_allowed),
      Intent(priority)
{
}

std::string DribbleIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

bool DribbleIntent::operator==(const DribbleIntent &other) const
{
    return DribblePrimitive::operator==(other) && Intent::operator==(other);
}

bool DribbleIntent::operator!=(const DribbleIntent &other) const
{
    return !((*this) == other);
}

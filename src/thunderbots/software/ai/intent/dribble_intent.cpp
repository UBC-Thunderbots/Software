#include "ai/intent/dribble_intent.h"
#include "ai/intent/visitor/intent_visitor.h"

const std::string DribbleIntent::INTENT_NAME = "Dribble Intent";

DribbleIntent::DribbleIntent(unsigned int robot_id, const Point &dest,
                             const Angle &final_angle, double final_speed, double rpm,
                             bool small_kick_allowed) :
                             DribblePrimitive(robot_id, dest, final_angle, final_speed, rpm, small_kick_allowed)
{
}

std::string DribbleIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

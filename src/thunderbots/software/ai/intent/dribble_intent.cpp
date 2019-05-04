#include "ai/intent/dribble_intent.h"

#include "ai/intent/visitor/intent_visitor.h"

const std::string DribbleIntent::INTENT_NAME = "Dribble Intent";

DribbleIntent::DribbleIntent(unsigned int robot_id, const Point &dest,
                             const Angle &final_angle, double rpm,
                             bool small_kick_allowed, unsigned int priority)
    : DribblePrimitive(robot_id, dest, final_angle, rpm, small_kick_allowed),
      Intent(priority)
{
}

std::string DribbleIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

void DribbleIntent::accept(IntentVisitor &visitor) const
{
    visitor.visit(*this);
}

bool DribbleIntent::operator==(const DribbleIntent &other) const
{
    return DribblePrimitive::operator==(other) && Intent::operator==(other);
}

bool DribbleIntent::operator!=(const DribbleIntent &other) const
{
    return !((*this) == other);
}

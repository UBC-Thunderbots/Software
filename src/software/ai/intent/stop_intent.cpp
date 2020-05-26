#include "software/ai/intent/stop_intent.h"

const std::string StopIntent::INTENT_NAME = "Stop Intent";

StopIntent::StopIntent(unsigned int robot_id, bool coast, unsigned int priority)
    : StopPrimitive(robot_id, coast), Intent(priority)
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
    return StopPrimitive::operator==(other) && Intent::operator==(other);
}

bool StopIntent::operator!=(const StopIntent &other) const
{
    return !((*this) == other);
}

#include "ai/intent/stop_intent.h"

const std::string StopIntent::INTENT_NAME = "Stop Intent";

StopIntent::StopIntent(unsigned int robot_id, bool coast)
        : StopPrimitive(robot_id, coast)
{
}

std::string StopIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

bool StopIntent::operator==(const StopIntent &other) const {
    return StopPrimitive::operator==(other) && Intent::operator==(other);
}

bool StopIntent::operator!=(const StopIntent &other) const {
    return !((*this) == other);
}

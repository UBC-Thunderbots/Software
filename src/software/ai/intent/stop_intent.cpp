#include "software/ai/intent/stop_intent.h"

const std::string StopIntent::INTENT_NAME = "Stop Intent";

StopIntent::StopIntent(unsigned int robot_id, bool coast, unsigned int priority)
    : DirectPrimitiveIntent(
          robot_id, priority,
          ProtoCreatorPrimitiveVisitor().createPrimitive(StopPrimitive(robot_id, coast)))
{
}

std::string StopIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

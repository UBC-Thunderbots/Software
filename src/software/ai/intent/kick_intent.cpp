#include "software/ai/intent/kick_intent.h"

const std::string KickIntent::INTENT_NAME = "Kick Intent";

KickIntent::KickIntent(unsigned int robot_id, const Point &kick_origin,
                       const Angle &kick_direction, double kick_speed_meters_per_second,
                       unsigned int priority)
    : DirectPrimitiveIntent(
          robot_id, priority,
          ProtoCreatorPrimitiveVisitor().createPrimitive(KickPrimitive(
              robot_id, kick_origin, kick_direction, kick_speed_meters_per_second)))
{
}

std::string KickIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

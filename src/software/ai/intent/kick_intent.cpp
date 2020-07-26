#include "software/ai/intent/kick_intent.h"

const std::string KickIntent::INTENT_NAME = "Kick Intent";

KickIntent::KickIntent(unsigned int robot_id, const Point &kick_origin,
                       const Angle &kick_direction, double kick_speed_meters_per_second,
                       unsigned int priority)
    : Intent(robot_id,
             ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(KickPrimitive(
                 robot_id, kick_origin, kick_direction, kick_speed_meters_per_second)),
             priority)

{
}

std::string KickIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

bool KickIntent::operator==(const KickIntent &other) const
{
    return Intent::operator==(other);
}

bool KickIntent::operator!=(const KickIntent &other) const
{
    return !((*this) == other);
}

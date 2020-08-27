#include "software/ai/intent/kick_intent.h"

KickIntent::KickIntent(unsigned int robot_id, const Point &kick_origin,
                       const Angle &kick_direction, double kick_speed_meters_per_second,
                       unsigned int priority)
    : DirectPrimitiveIntent(
          robot_id, priority,
          *createKickPrimitive(kick_origin, kick_direction, kick_speed_meters_per_second))
{
}

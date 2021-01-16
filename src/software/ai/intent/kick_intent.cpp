#include "software/ai/intent/kick_intent.h"

KickIntent::KickIntent(unsigned int robot_id, const Point &kick_origin,
                       const Angle &kick_direction, double kick_speed_meters_per_second)
    : DirectPrimitiveIntent(robot_id, *createKickPrimitive(kick_origin, kick_direction,
                                                           kick_speed_meters_per_second))
{
}

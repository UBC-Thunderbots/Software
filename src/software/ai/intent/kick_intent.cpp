#include "software/ai/intent/kick_intent.h"

KickIntent::KickIntent(unsigned int robot_id, const Point &kick_origin,
                       const Angle &kick_direction, double kick_speed_meters_per_second)
    : DirectPrimitiveIntent(
          robot_id,
          *createMovePrimitive(
              kick_origin, 0, kick_direction, DribblerMode::OFF,
              AutoChipOrKick{AutoChipOrKickMode::AUTOKICK, kick_speed_meters_per_second},
              MaxAllowedSpeedMode::PHYSICAL_LIMIT))
{
}

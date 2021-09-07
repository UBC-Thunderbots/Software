#include "software/ai/intent/chip_intent.h"

ChipIntent::ChipIntent(unsigned int robot_id, const Point &chip_origin,
                       const Angle &chip_direction, double chip_distance_meters,
                       RobotConstants_t robot_constants)
    : DirectPrimitiveIntent(
          robot_id,
          *createMovePrimitive(
              chip_origin, 0, chip_direction, DribblerMode::OFF,
              AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, chip_distance_meters},
              MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0, robot_constants))
{
}

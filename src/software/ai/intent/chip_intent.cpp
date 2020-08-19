#include "software/ai/intent/chip_intent.h"

const std::string ChipIntent::INTENT_NAME = "Chip Intent";

ChipIntent::ChipIntent(unsigned int robot_id, const Point &chip_origin,
                       const Angle &chip_direction, double chip_distance_meters,
                       unsigned int priority)
    : DirectPrimitiveIntent(
          robot_id, priority,
          ProtoCreatorPrimitiveVisitor().createPrimitive(
              ChipPrimitive(robot_id, chip_origin, chip_direction, chip_distance_meters)))
{
}

std::string ChipIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

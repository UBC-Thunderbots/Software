#include "software/ai/intent/chip_intent.h"

ChipIntent::ChipIntent(unsigned int robot_id, const Point &chip_origin,
                       const Angle &chip_direction, double chip_distance_meters)
    : DirectPrimitiveIntent(robot_id, *createChipPrimitive(chip_origin, chip_direction,
                                                           chip_distance_meters))
{
}

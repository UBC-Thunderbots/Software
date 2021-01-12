#pragma once

#include "software/ai/intent/direct_primitive_intent.h"
#include "software/geom/angle.h"
#include "software/geom/point.h"

class ChipIntent : public DirectPrimitiveIntent
{
   public:
    /**
     * Creates a new Chip Intent
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param chip_origin The location where the chip will be taken
     * @param chip_direction The orientation the Robot will chip at
     * @param chip_distance_meters The distance between the starting location
     * of the chip and the location of the first bounce
     */
    explicit ChipIntent(unsigned int robot_id, const Point& chip_origin,
                        const Angle& chip_direction, double chip_distance_meters);

    ChipIntent() = delete;
};

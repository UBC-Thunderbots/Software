#pragma once

#include "software/ai/intent/direct_primitive_intent.h"
#include "software/geom/angle.h"
#include "software/geom/point.h"
#include "software/primitive/chip_primitive.h"

class ChipIntent : public DirectPrimitiveIntent
{
   public:
    static const std::string INTENT_NAME;
    /**
     * Creates a new Chip Intent
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param chip_origin The location where the chip will be taken
     * @param chip_direction The orientation the Robot will chip at
     * @param chip_distance_meters The distance between the starting location
     * of the chip and the location of the first bounce
     * @param priority The priority of this Intent. A larger number indicates a higher
     * priority
     */
    explicit ChipIntent(unsigned int robot_id, const Point& chip_origin,
                        const Angle& chip_direction, double chip_distance_meters,
                        unsigned int priority);

    std::string getIntentName(void) const override;
};

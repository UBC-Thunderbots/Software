#pragma once

#include "software/ai/intent/intent.h"
#include "software/geom/angle.h"
#include "software/geom/point.h"
#include "software/primitive/chip_primitive.h"

class ChipIntent : public ChipPrimitive, public Intent
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

    void accept(IntentVisitor& visitor) const override;

    /**
     * Compares ChipIntents for equality. ChipIntents are considered equal if all
     * their member variables are equal.
     *
     * @param other the ChipIntents to compare with for equality
     * @return true if the ChipIntents are equal and false otherwise
     */
    bool operator==(const ChipIntent& other) const;

    /**
     * Compares ChipIntents for inequality.
     *
     * @param other the ChipIntent to compare with for inequality
     * @return true if the ChipIntents are not equal and false otherwise
     */
    bool operator!=(const ChipIntent& other) const;
};

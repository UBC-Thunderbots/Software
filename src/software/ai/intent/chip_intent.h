#pragma once

#include "software/ai/intent/intent.h"
#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/primitive/chip_primitive.h"

class ChipIntent : public ChipPrimitive, public Intent
{
   public:
    static const std::string INTENT_NAME;
    /**
     * Creates a new Chip Intent
     *
     * @param robot_id The id of the robot that this Intent is for
     * @param dest The destination of the Movement
     * @param final_angle The final angle the robot should have at the end of the movement
     * @param final_speed The final speed the robot should have when it arrives at its
     * destination
     * @param priority The priority of this Intent. A larger number indicates a higher
     * priority
     */
    explicit ChipIntent(unsigned int robot_id, const Point& dest,
                        const Angle& final_angle, double final_speed,
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

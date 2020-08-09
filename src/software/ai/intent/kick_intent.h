#pragma once

#include "software/ai/intent/direct_primitive_intent.h"
#include "software/geom/angle.h"
#include "software/geom/point.h"
#include "software/primitive/kick_primitive.h"

class KickIntent : public DirectPrimitiveIntent
{
   public:
    static const std::string INTENT_NAME;
    /**
     * Creates a new Kick Intent
     *
     * @param robot_id The id of the robot that this Intent is for
     * @param dest The destination of the Movement
     * @param final_angle The final angle the robot should have at the end of the movement
     * @param final_speed The final speed the robot should have when it arrives at its
     * destination
     * @param priority The priority of this Intent. A larger number indicates a higher
     * priority
     */
    explicit KickIntent(unsigned int robot_id, const Point& dest,
                        const Angle& final_angle, double final_speed,
                        unsigned int priority);

    std::string getIntentName(void) const override;
};

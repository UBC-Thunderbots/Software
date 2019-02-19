#pragma once

#include "ai/intent/intent.h"
#include "ai/primitive/kick_primitive.h"
#include "geom/angle.h"
#include "geom/point.h"

class KickIntent : public Intent, public KickPrimitive
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
     */
    explicit KickIntent(unsigned int robot_id, const Point &dest,
                        const Angle &final_angle, double final_speed);

    std::string getIntentName(void) const override;
};

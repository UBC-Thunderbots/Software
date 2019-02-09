#pragma once

#include "ai/intent/intent.h"
#include "ai/primitive/catch_primitive.h"
#include "geom/angle.h"
#include "geom/point.h"

class CatchIntent : public Intent, public CatchPrimitive
{
   public:
    static const std::string INTENT_NAME;
    /**
     * Creates a new Catch Intent
     *
     * @param robot_id The id of the robot that this Intent is for
     * @param velocity Velocity to move robot forwards/backwards
     * to catch the ball without it bouncing off the dribbler; units are m/s
     * @param dribbler_rpm Speed to rotate dribbler at. Units are RPM.
     * @param ball_intercept_margin A scaling factor for how far in front of the ball to
     * make the point of intercept. It scales based on the difference in velocity between
     * the robot and the ball.
     */
    explicit CatchIntent(unsigned int robot_id, double velocity, double dribbler_rpm,
                         double ball_intercept_margin);

    std::string getIntentName(void) const override;
};

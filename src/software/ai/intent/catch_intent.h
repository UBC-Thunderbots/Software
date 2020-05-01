#pragma once

#include "software/ai/intent/intent.h"
#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/primitive/catch_primitive.h"

class CatchIntent : public CatchPrimitive, public Intent
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
     * @param priority The priority of this Intent. A larger number indicates a higher
     * priority
     */
    explicit CatchIntent(unsigned int robot_id, double velocity, double dribbler_rpm,
                         double ball_intercept_margin, unsigned int priority);

    std::string getIntentName(void) const override;

    void accept(IntentVisitor& visitor) const override;

    /**
     * Compares CatchIntents for equality. CatchIntents are considered equal if all
     * their member variables are equal.
     *
     * @param other the CatchIntents to compare with for equality
     * @return true if the CatchIntents are equal and false otherwise
     */
    bool operator==(const CatchIntent& other) const;

    /**
     * Compares CatchIntents for inequality.
     *
     * @param other the CatchIntent to compare with for inequality
     * @return true if the CatchIntents are not equal and false otherwise
     */
    bool operator!=(const CatchIntent& other) const;
};

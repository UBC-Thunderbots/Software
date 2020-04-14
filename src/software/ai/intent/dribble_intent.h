#pragma once

#include "software/ai/intent/intent.h"
#include "software/new_geom/angle.h"
#include "software/new_geom/point.h"
#include "software/primitive/dribble_primitive.h"

class DribbleIntent : public DribblePrimitive, public Intent
{
   public:
    static const std::string INTENT_NAME;
    /**
     * Creates a new Dribble Intent
     *
     * Moves the robot in a straight line between its current position and the given
     * destination with the dribbler on
     *
     * @param robot_id The id of the Robot to run this Intent
     * @param dest The final destination of the movement
     * @param final_angle The final orientation the robot should have at the end
     * of the movement
     * @param final_speed The final speed the Robot should have when it reaches
     * its destination at the end of the movement
     * @param rpm The rotation speed of the dribbler in RPMs
     * @param small_kick_allowed Boolean of whether the robot is allowed o do a small
     * kick while dribbling in order to release the ball. This is due to the rule that
     * a robot may not dribble more than 1 meter without releasing the ball. This is not
     * obeyed in firmware
     * @param priority The priority of this Intent. A larger number indicates a higher
     * priority
     *
     */
    explicit DribbleIntent(unsigned int robot_id, const Point& dest,
                           const Angle& final_angle, double rpm, bool small_kick_allowed,
                           unsigned int priority);

    std::string getIntentName(void) const override;

    void accept(IntentVisitor& visitor) const override;

    /**
     * Compares DribbleIntents for equality. DribbleIntents are considered equal if all
     * their member variables are equal.
     *
     * @param other the DribbleIntents to compare with for equality
     * @return true if the DribbleIntents are equal and false otherwise
     */
    bool operator==(const DribbleIntent& other) const;

    /**
     * Compares DribbleIntents for inequality.
     *
     * @param other the DribbleIntent to compare with for inequality
     * @return true if the DribbleIntents are not equal and false otherwise
     */
    bool operator!=(const DribbleIntent& other) const;
};

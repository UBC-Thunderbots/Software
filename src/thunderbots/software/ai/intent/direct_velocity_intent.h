#pragma once

#include "ai/intent/intent.h"
#include "ai/primitive/direct_velocity_primitive.h"
#include "geom/angle.h"
#include "geom/point.h"

class DirectVelocityIntent : public Intent, public DirectVelocityPrimitive
{
   public:
    static const std::string INTENT_NAME;
    /**
     * Creates a new DirectVelocity Intent
     *
     * @param robot_id The id of the robot that this Intent is for
     * @param x_velocity positive forward
     * @param y_velocity positive forward
     * @param angular_velocity positive clockwise
     * @param dribbler_rpm The dribbler speed in rpm
     * @param priority The priority of this Intent. A larger number indicates a higher
     * priority
     */
    explicit DirectVelocityIntent(unsigned int robot_id, double x_velocity,
                                  double y_velocity, double angular_velocity,
                                  double dribbler_rpm, unsigned int priority);

    std::string getIntentName(void) const override;

    /**
     * Compares DirectVelocityIntents for equality. DirectVelocityIntents are considered
     * equal if all their member variables are equal.
     *
     * @param other the DirectVelocityIntents to compare with for equality
     * @return true if the DirectVelocityIntents are equal and false otherwise
     */
    bool operator==(const DirectVelocityIntent& other) const;

    /**
     * Compares DirectVelocityIntents for inequality.
     *
     * @param other the DirectVelocityIntent to compare with for inequality
     * @return true if the DirectVelocityIntents are not equal and false otherwise
     */
    bool operator!=(const DirectVelocityIntent& other) const;
};

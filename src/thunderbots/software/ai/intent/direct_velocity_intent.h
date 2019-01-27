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
     */
    explicit DirectVelocityIntent(unsigned int robot_id, double x_velocity,
                                  double y_velocity, double angular_velocity,
                                  double dribbler_rpm);

    std::string getIntentName(void) const override;
};

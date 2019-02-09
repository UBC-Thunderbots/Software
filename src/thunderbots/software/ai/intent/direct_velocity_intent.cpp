#include "direct_velocity_intent.h"

const std::string DirectVelocityIntent::INTENT_NAME = "Direct Velocity Intent";

DirectVelocityIntent::DirectVelocityIntent(unsigned int robot_id, double x_velocity,
                                           double y_velocity, double angular_velocity,
                                           double dribbler_rpm)
    : DirectVelocityPrimitive(robot_id, x_velocity, y_velocity, angular_velocity,
                              dribbler_rpm)
{
}

std::string DirectVelocityIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

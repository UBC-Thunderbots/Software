#include "software/ai/intent/direct_velocity_intent.h"

const std::string DirectVelocityIntent::INTENT_NAME = "Direct Velocity Intent";

DirectVelocityIntent::DirectVelocityIntent(unsigned int robot_id, double x_velocity,
                                           double y_velocity, double angular_velocity,
                                           double dribbler_rpm, unsigned int priority)
    : DirectVelocityPrimitive(robot_id, x_velocity, y_velocity, angular_velocity,
                              dribbler_rpm),
      Intent(priority)
{
}

void DirectVelocityIntent::accept(IntentVisitor &visitor) const
{
    visitor.visit(*this);
}

std::string DirectVelocityIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

bool DirectVelocityIntent::operator==(const DirectVelocityIntent &other) const
{
    return DirectVelocityPrimitive::operator==(other) && Intent::operator==(other);
}

bool DirectVelocityIntent::operator!=(const DirectVelocityIntent &other) const
{
    return !((*this) == other);
}

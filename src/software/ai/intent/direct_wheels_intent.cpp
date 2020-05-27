#include "software/ai/intent/direct_wheels_intent.h"

const std::string DirectWheelsIntent::INTENT_NAME = "Direct Wheels Intent";

DirectWheelsIntent::DirectWheelsIntent(unsigned int robot_id,
                                       int16_t front_left_wheel_power,
                                       int16_t back_left_wheel_power,
                                       int16_t front_right_wheel_power,
                                       int16_t back_right_wheel_power,
                                       double dribbler_rpm, unsigned int priority)
    : DirectWheelsPrimitive(robot_id, front_left_wheel_power, back_left_wheel_power,
                            front_right_wheel_power, back_right_wheel_power,
                            dribbler_rpm),
      Intent(priority)
{
}

void DirectWheelsIntent::accept(IntentVisitor &visitor) const
{
    visitor.visit(*this);
}

std::string DirectWheelsIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

bool DirectWheelsIntent::operator==(const DirectWheelsIntent &other) const
{
    return DirectWheelsPrimitive::operator==(other) && Intent::operator==(other);
}

bool DirectWheelsIntent::operator!=(const DirectWheelsIntent &other) const
{
    return !((*this) == other);
}

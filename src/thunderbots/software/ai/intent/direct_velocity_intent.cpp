#include "direct_velocity_intent.h"

DirectVelocityIntent::DirectVelocityIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed, int priority)
    : DirectVelocityPrimitive(robot_id, dest, final_angle, final_speed), priority(priority) 
{
}


DirectVelocityIntent::DirectVelocityIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed)
    : DirectVelocityPrimitive(robot_id, dest, final_angle, final_speed)
{
}

std::string DirectVelocityIntent::getIntentName() const
{
    return DIRECT_VELOCITY_INTENT_NAME;
}

int DirectVelocityIntent::getPriority() const
{
    return priority;
}

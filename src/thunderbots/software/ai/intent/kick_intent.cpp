#include "kick_intent.h"

KickIntent::KickIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed, int priority)
    : KickPrimitive(robot_id, dest, final_angle, final_speed), priority(priority) 
{
}


KickIntent::KickIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed)
    : KickPrimitive(robot_id, dest, final_angle, final_speed)
{
}

std::string KickIntent::getIntentName() const
{
    return KICK_INTENT_NAME;
}

int KickIntent::getPriority() const
{
    return priority;
}

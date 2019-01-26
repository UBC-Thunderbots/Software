#include "catch_intent.h"

CatchIntent::CatchIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed, int priority)
    : CatchPrimitive(robot_id, dest, final_angle, final_speed), priority(priority) 
{
}


CatchIntent::CatchIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed)
    : CatchPrimitive(robot_id, dest, final_angle, final_speed)
{
}

std::string CatchIntent::getIntentName() const
{
    return CATCH_INTENT_NAME;
}

int CatchIntent::getPriority() const
{
    return priority;
}

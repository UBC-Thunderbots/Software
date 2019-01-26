#include "pivot_intent.h"

PivotIntent::PivotIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed, int priority)
    : PivotPrimitive(robot_id, dest, final_angle, final_speed), priority(priority) 
{
}


PivotIntent::PivotIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed)
    : PivotPrimitive(robot_id, dest, final_angle, final_speed)
{
}

std::string PivotIntent::getIntentName() const
{
    return PIVOT_INTENT_NAME;
}

int PivotIntent::getPriority() const
{
    return priority;
}

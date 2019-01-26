#include "move_intent.h"

MoveIntent::MoveIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed, int priority)
    : MovePrimitive(robot_id, dest, final_angle, final_speed), priority(priority) 
{
}


MoveIntent::MoveIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed)
    : MovePrimitive(robot_id, dest, final_angle, final_speed)
{
}

std::string MoveIntent::getIntentName() const
{
    return MOVE_INTENT_NAME;
}

int MoveIntent::getPriority() const
{
    return priority;
}

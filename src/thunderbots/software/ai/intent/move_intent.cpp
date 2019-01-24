#include "move_intent.h"

MoveIntent::MoveIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed, int priority)
    : robot_id(robot_id), dest(dest), final_angle(final_angle), final_speed(final_speed), priority(priority), 
                       MovePrimitive(robot_id, dest, final_angle, final_speed) 
{
}


MoveIntent::MoveIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed)
    : robot_id(robot_id), dest(dest), final_angle(final_angle), final_speed(final_speed),
                       MovePrimitive(robot_id, dest, final_angle, final_speed) 
{
}

unsigned int MoveIntent::getRobotId() const
{
    return robot_id;
}

std::string MoveIntent::getIntentName() const
{
    return MOVE_INTENT_NAME;
}

Point MoveIntent::getDestination() const
{
    return dest;
}

Angle MoveIntent::getFinalAngle() const
{
    return final_angle;
}

double MoveIntent::getFinalSpeed() const
{
    return final_speed;
}

int MoveIntent::getPriority() const
{
    return priority;
}

#include "ai/intent/movespin_intent.h"

const std::string MoveSpinIntent::INTENT_NAME = "MoveSpin Intent";

MoveSpinIntent::MoveSpinIntent(unsigned int robot_id, const Point &dest, const AngularVelocity &angular_vel)
        : MoveSpinPrimitive(robot_id, dest, angular_vel)
{
}

std::string MoveSpinIntent::getIntentName(void) const
{
    return INTENT_NAME;
}


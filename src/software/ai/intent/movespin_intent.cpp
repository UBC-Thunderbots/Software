#include "software/ai/intent/movespin_intent.h"

const std::string MoveSpinIntent::INTENT_NAME = "MoveSpin Intent";

MoveSpinIntent::MoveSpinIntent(unsigned int robot_id, const Point &dest,
                               const AngularVelocity &angular_vel, double final_speed,
                               unsigned int priority)
    : Intent(robot_id,
             ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(
                 MoveSpinPrimitive(robot_id, dest, angular_vel, final_speed)),
             priority)

{
}

std::string MoveSpinIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

bool MoveSpinIntent::operator==(const MoveSpinIntent &other) const
{
    return Intent::operator==(other);
}

bool MoveSpinIntent::operator!=(const MoveSpinIntent &other) const
{
    return !((*this) == other);
}

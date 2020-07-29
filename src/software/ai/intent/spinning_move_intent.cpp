#include "software/ai/intent/spinning_move_intent.h"

const std::string SpinningMoveIntent::INTENT_NAME = "Spinning Move Intent";

SpinningMoveIntent::SpinningMoveIntent(unsigned int robot_id, const Point &dest,
                                       const AngularVelocity &angular_vel,
                                       double final_speed, unsigned int priority)
    : Intent(robot_id,
             ProtoCreatorPrimitiveVisitor().createPrimitiveMsg(
                 SpinningMovePrimitive(robot_id, dest, angular_vel, final_speed)),
             priority)
{
}

std::string SpinningMoveIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

bool SpinningMoveIntent::operator==(const SpinningMoveIntent &other) const
{
    return Intent::operator==(other);
}

bool SpinningMoveIntent::operator!=(const SpinningMoveIntent &other) const
{
    return !((*this) == other);
}

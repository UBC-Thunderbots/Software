#include "software/ai/intent/spinning_move_intent.h"

const std::string SpinningMoveIntent::INTENT_NAME = "Spinning Move Intent";

SpinningMoveIntent::SpinningMoveIntent(unsigned int robot_id, const Point &dest,
                                       const AngularVelocity &angular_vel,
                                       double final_speed, unsigned int priority)
    : SpinningMovePrimitive(robot_id, dest, angular_vel, final_speed), Intent(priority)
{
}

std::string SpinningMoveIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

void SpinningMoveIntent::accept(IntentVisitor &visitor) const
{
    visitor.visit(*this);
}

bool SpinningMoveIntent::operator==(const SpinningMoveIntent &other) const
{
    return SpinningMovePrimitive::operator==(other) && Intent::operator==(other);
}

bool SpinningMoveIntent::operator!=(const SpinningMoveIntent &other) const
{
    return !((*this) == other);
}

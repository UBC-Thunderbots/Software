#include "ai/intent/movespin_intent.h"

#include "ai/intent/visitor/intent_visitor.h"

const std::string MoveSpinIntent::INTENT_NAME = "MoveSpin Intent";

MoveSpinIntent::MoveSpinIntent(unsigned int robot_id, const Point &dest,
                               const AngularVelocity &angular_vel, unsigned int priority)
    : MoveSpinPrimitive(robot_id, dest, angular_vel), Intent(priority)
{
}

std::string MoveSpinIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

void MoveSpinIntent::accept(IntentVisitor &visitor) const
{
    visitor.visit(*this);
}

bool MoveSpinIntent::operator==(const MoveSpinIntent &other) const
{
    return MoveSpinPrimitive::operator==(other) && Intent::operator==(other);
}

bool MoveSpinIntent::operator!=(const MoveSpinIntent &other) const
{
    return !((*this) == other);
}

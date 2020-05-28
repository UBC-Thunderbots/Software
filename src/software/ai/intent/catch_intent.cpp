#include "software/ai/intent/catch_intent.h"

const std::string CatchIntent::INTENT_NAME = "Catch Intent";

CatchIntent::CatchIntent(unsigned int robot_id, double velocity, double dribbler_rpm,
                         double ball_intercept_margin, unsigned int priority)
    : CatchPrimitive(robot_id, velocity, dribbler_rpm, ball_intercept_margin),
      Intent(priority)
{
}

std::string CatchIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

void CatchIntent::accept(IntentVisitor &visitor) const
{
    visitor.visit(*this);
}

bool CatchIntent::operator==(const CatchIntent &other) const
{
    return CatchPrimitive::operator==(other) && Intent::operator==(other);
}

bool CatchIntent::operator!=(const CatchIntent &other) const
{
    return !((*this) == other);
}

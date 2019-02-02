#include "catch_intent.h"

const std::string CatchIntent::INTENT_NAME = "Catch Intent";

CatchIntent::CatchIntent(unsigned int robot_id, double velocity, double dribbler_rpm,
                         double ball_intercept_margin)
    : CatchPrimitive(robot_id, velocity, dribbler_rpm, ball_intercept_margin)
{
}

std::string CatchIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

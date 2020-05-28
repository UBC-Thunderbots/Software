#include "software/ai/intent/pivot_intent.h"

const std::string PivotIntent::INTENT_NAME = "Pivot Intent";

PivotIntent::PivotIntent(unsigned int robot_id, const Point &pivot_point,
                         const Angle &final_angle, const Angle &pivot_speed,
                         bool enable_dribbler, unsigned int priority)
    : PivotPrimitive(robot_id, pivot_point, final_angle, pivot_speed, enable_dribbler),
      Intent(priority)
{
}

std::string PivotIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

void PivotIntent::accept(IntentVisitor &visitor) const
{
    visitor.visit(*this);
}

bool PivotIntent::operator==(const PivotIntent &other) const
{
    return PivotPrimitive::operator==(other) && Intent::operator==(other);
}

bool PivotIntent::operator!=(const PivotIntent &other) const
{
    return !((*this) == other);
}

#include "pivot_intent.h"

const std::string PivotIntent::INTENT_NAME = "Pivot Intent";

PivotIntent::PivotIntent(unsigned int robot_id, const Point &pivot_point,
                         const Angle &final_angle, const double pivot_radius,
                         unsigned int priority)
    : PivotPrimitive(robot_id, pivot_point, final_angle, pivot_radius), Intent(priority)
{
}

std::string PivotIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

bool PivotIntent::operator==(const PivotIntent &other) const
{
    return PivotPrimitive::operator==(other) && Intent::operator==(other);
}

bool PivotIntent::operator!=(const PivotIntent &other) const
{
    return !((*this) == other);
}

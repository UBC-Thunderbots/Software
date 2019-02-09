#include "pivot_intent.h"

const std::string PivotIntent::INTENT_NAME = "Pivot Intent";

PivotIntent::PivotIntent(unsigned int robot_id, const Point &pivot_point,
                         const Angle &final_angle, const double pivot_radius)
    : PivotPrimitive(robot_id, pivot_point, final_angle, pivot_radius)
{
}

std::string PivotIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

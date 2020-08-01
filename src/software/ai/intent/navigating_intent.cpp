#include "software/ai/intent/navigating_intent.h"

NavigatingIntent::NavigatingIntent(unsigned int robot_id, PrimitiveMsg primitive_msg,
                                   unsigned int priority,
                                   NavigatorParams navigator_params)
    : Intent(robot_id, primitive_msg, priority), navigator_params(navigator_params)
{
}

void NavigatingIntent::setMotionConstraints(
    const std::set<MotionConstraint>& motion_constraints)
{
    navigator_params.motion_constraints = motion_constraints;
}

std::optional<NavigatorParams> NavigatingIntent::getNavigatorParams() const
{
    return navigator_params;
}

bool NavigatingIntent::operator==(const NavigatingIntent& other) const
{
    return Intent::operator==(other) && navigator_params == other.navigator_params;
}

bool NavigatingIntent::operator!=(const NavigatingIntent& other) const
{
    return !((*this) == other);
}

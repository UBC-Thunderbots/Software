#include "software/ai/intent/navigating_intent.h"

NavigatingIntent::NavigatingIntent(unsigned int robot_id, PrimitiveMsg primitive_msg,
                                   unsigned int priority, const Point& destination,
                                   const Angle& final_angle, double final_speed,
                                   BallCollisionType ball_collision_type)
    : Intent(robot_id, primitive_msg, priority),
      navigator_params(
          NavigatorParams{.destination         = destination,
                          .final_speed         = final_speed,
                          .final_angle         = final_angle,
                          .ball_collision_type = ball_collision_type,
                          .motion_constraints  = std::set<MotionConstraint>()})
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

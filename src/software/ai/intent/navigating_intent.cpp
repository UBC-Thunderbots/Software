#include "software/ai/intent/navigating_intent.h"

NavigatingIntent::NavigatingIntent(unsigned int robot_id, unsigned int priority,
                    Point destination, BallCollisionType ball_collision_type )
    : Intent(robot_id,priority),
      navigation_destination(destination),
      ball_collision_type(ball_collision_type) { }

const Point &NavigatingIntent::getNavigationDestination() const
{
    return navigation_destination;
}

const BallCollisionType &NavigatingIntent::getBallCollisionType() const
{
    return ball_collision_type;
}

void NavigatingIntent::accept(IntentVisitor &visitor) const
{
    visitor.visit(*this);
}

bool NavigatingIntent::operator==(const NavigatingIntent &other) const
{
    return Intent::operator==(other) &&
           navigation_destination == other.navigation_destination &&
           ball_collision_type == other.ball_collision_type;
}

bool NavigatingIntent::operator!=(const NavigatingIntent &other) const
{
    return !((*this) == other);
}

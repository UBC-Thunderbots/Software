#include "software/ai/intent/navigating_intent.h"

NavigatingIntent::NavigatingIntent(unsigned int robot_id, unsigned int priority,
                                   Point destination, double final_speed,
                                   BallCollisionType ball_collision_type)
    : Intent(robot_id, priority),
      destination(destination),
      final_speed(final_speed),
      ball_collision_type(ball_collision_type)
{
}

const Point &NavigatingIntent::getDestination() const
{
    return destination;
}

const BallCollisionType &NavigatingIntent::getBallCollisionType() const
{
    return ball_collision_type;
}

double NavigatingIntent::getFinalSpeed() const
{
    return final_speed;
}

bool NavigatingIntent::operator==(const NavigatingIntent &other) const
{
    return Intent::operator==(other) && destination == other.destination &&
           final_speed == other.final_speed &&
           ball_collision_type == other.ball_collision_type;
}

bool NavigatingIntent::operator!=(const NavigatingIntent &other) const
{
    return !((*this) == other);
}

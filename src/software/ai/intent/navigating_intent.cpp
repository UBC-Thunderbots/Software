#include "software/ai/intent/navigating_intent.h"

NavigatingIntent::NavigatingIntent(unsigned int robot_id, Point destination,
                                   double final_speed,
                                   BallCollisionType ball_collision_type,
                                   MaxAllowedSpeedMode max_allowed_speed_mode)
    : Intent(robot_id),
      destination(destination),
      final_speed(final_speed),
      ball_collision_type(ball_collision_type),
      max_allowed_speed_mode(max_allowed_speed_mode)
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

MaxAllowedSpeedMode NavigatingIntent::getMaxAllowedSpeedMode() const
{
    return max_allowed_speed_mode;
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

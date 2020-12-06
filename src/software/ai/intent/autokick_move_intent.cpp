#include "software/ai/intent/autokick_move_intent.h"

AutokickMoveIntent::AutokickMoveIntent(unsigned int robot_id, const Point &destination,
                                       const Angle &final_angle, double final_speed,
                                       DribblerMode dribbler_mode,
                                       double kick_speed_meters_per_second,
                                       BallCollisionType ball_collision_type)
    : NavigatingIntent(robot_id, destination, final_speed, ball_collision_type),
      final_angle(final_angle),
      dribbler_mode(dribbler_mode),
      kick_speed_meters_per_second(kick_speed_meters_per_second)
{
}

void AutokickMoveIntent::accept(IntentVisitor &visitor) const
{
    visitor.visit(*this);
}

void AutokickMoveIntent::accept(NavigatingIntentVisitor &visitor) const
{
    visitor.visit(*this);
}

const Angle &AutokickMoveIntent::getFinalAngle() const
{
    return final_angle;
}

const DribblerMode &AutokickMoveIntent::getDribblerMode() const
{
    return dribbler_mode;
}

double AutokickMoveIntent::getKickSpeed() const
{
    return kick_speed_meters_per_second;
}

bool AutokickMoveIntent::operator==(const AutokickMoveIntent &other) const
{
    return NavigatingIntent::operator==(other) &&
           this->final_angle == other.final_angle &&
           this->kick_speed_meters_per_second == other.kick_speed_meters_per_second &&
           this->dribbler_mode == other.dribbler_mode;
}

bool AutokickMoveIntent::operator!=(const AutokickMoveIntent &other) const
{
    return !((*this) == other);
}

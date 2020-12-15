#include "software/ai/intent/autokick_move_intent.h"

AutokickMoveIntent::AutokickMoveIntent(unsigned int robot_id, const Point &destination,
                                       const Angle &final_angle, double final_speed,
                                       DribblerMode dribbler_mode,
                                       double kick_speed_meters_per_second,
                                       BallCollisionType ball_collision_type)
    : MoveIntent(robot_id, destination, final_angle, final_speed, dribbler_mode,
                 ball_collision_type),
      kick_speed_meters_per_second(kick_speed_meters_per_second)
{
}

AutokickMoveIntent::AutokickMoveIntent(MoveIntent move_intent,
                                       double kick_speed_meters_per_second)
    : MoveIntent(move_intent), kick_speed_meters_per_second(kick_speed_meters_per_second)
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

double AutokickMoveIntent::getKickSpeed() const
{
    return kick_speed_meters_per_second;
}

bool AutokickMoveIntent::operator==(const AutokickMoveIntent &other) const
{
    return MoveIntent::operator==(other) &&
           this->kick_speed_meters_per_second == other.kick_speed_meters_per_second;
}

bool AutokickMoveIntent::operator!=(const AutokickMoveIntent &other) const
{
    return !((*this) == other);
}

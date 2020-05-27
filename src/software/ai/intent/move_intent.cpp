#include "software/ai/intent/move_intent.h"

const std::string MoveIntent::INTENT_NAME = "Move Intent";

MoveIntent::MoveIntent(unsigned int robot_id, const Point &dest, const Angle &final_angle,
                       double final_speed, unsigned int priority,
                       DribblerEnable enable_dribbler, MoveType move_type,
                       AutokickType autokick, BallCollisionType ball_collision_type)
    : MovePrimitive(robot_id, dest, final_angle, final_speed, enable_dribbler, move_type,
                    autokick),
      Intent(priority),
      ball_collision_type(ball_collision_type)
{
}

std::string MoveIntent::getIntentName(void) const
{
    return INTENT_NAME;
}

BallCollisionType MoveIntent::getBallCollisionType() const
{
    return ball_collision_type;
}

void MoveIntent::accept(IntentVisitor &visitor) const
{
    visitor.visit(*this);
}

bool MoveIntent::operator==(const MoveIntent &other) const
{
    return MovePrimitive::operator==(other) && Intent::operator==(other);
}

bool MoveIntent::operator!=(const MoveIntent &other) const
{
    return !((*this) == other);
}
